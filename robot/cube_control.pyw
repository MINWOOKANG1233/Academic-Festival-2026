"""Windows panel for supervised three-object tilted pick trials."""
import json
from io import BytesIO
from pathlib import Path
import queue
import sys
import threading
import tkinter as tk
from tkinter import filedialog, ttk

from coordinate_pick import LeArmSerial
from object_pick import OBJECTS

ROOT = Path(__file__).resolve().parents[1]
deps = ROOT.parent / "work" / "camera-deps"
if deps.is_dir():
    sys.path.append(str(deps))

from PIL import Image, ImageTk
from demo_pick import GRASP_PRESETS, align_demo_depth, close_demo_object, demo_approach, demo_lower, run_demo
from side_pick import CameraStream, SidePick, Step


STAGES = {
    Step.HOME: ("초기 자세로", "집게를 비우고 초기 자세로 돌아가는 경로를 확인하세요."),
    Step.DETECT: ("카메라 좌표 읽기", "빈 배경 촬영 뒤 선택한 물체 하나를 세우고 손을 빼세요."),
    Step.APPROACH: ("대각선 접근", "화면에 표시된 물체가 맞고, 높은 접근 경로가 비었는지 확인하세요."),
    Step.LOWER: ("첫 사진 확인 · 파지 높이로", "물체와 집게 끝을 보고 앞뒤·좌우를 보정하세요. 열린 집게 접촉면 높이와 간격을 재고 하강 경로를 확인하세요."),
    Step.ALIGN: ("정렬 확인 · 짧게 전진", "사진에서 물체와 양쪽 집게 끝을 확인하세요. 사진과 실제 경로가 맞으면 전진하세요."),
    Step.CLOSE: ("양쪽 접촉 위치 확인 · 닫기", "물체가 집게 사이에 들어왔고 옆면을 잡을 높이인지 확인하세요."),
    Step.LIFT: ("10 mm 들어 올리기", "양쪽이 닿고 물체가 밀리거나 눌리지 않았는지 확인하세요."),
    Step.RELEASE: ("내려놓기 · 열린 채 후퇴", "들렸는지 결과를 선택하세요. 아래·뒤쪽·위쪽 경로가 비어야 해요."),
    Step.RETURN: ("초기 자세로 복귀", "물체가 빠졌고 초기 자세로 돌아갈 경로가 비었는지 확인하세요."),
    Step.STOPPED: ("새 시험 준비", "이후 명령이 중단됐어요. 팔과 집게를 확인하고 새 시험을 준비하세요."),
}

FIELDS = (
    ("camera", "카메라 포트", "COM3"), ("arm", "로봇 포트", "COM4"),
    ("forward", "초기 앞뒤 보정 mm", "0"), ("left", "초기 좌우 보정 mm", "0"),
    ("standoff", "접근 시 남길 거리 mm", "30"), ("rise", "대각선 상승량 mm", "30"),
    ("pitch", "파지 기울기 ° (아래로 −)", "-60"), ("roll", "손목 회전 서보", "497"),
    ("height", "집게 접촉면 실측 높이 mm", ""), ("opening", "열린 집게 실측 간격 mm", ""),
    ("height_trim", "파지 높이 보정 mm", "0"), ("insert", "마지막 전진 실측 거리 mm", ""),
    ("close", "닫힘 서보", "425"), ("jog", "미세 이동 mm", "5"),
    ("offset_x", "시험 앞뒤 오차 실측 mm", ""), ("offset_y", "시험 좌우 오차 실측 mm", ""),
)

PHOTOS = {"background": "빈 작업면", "observe": "초기 관찰", "approach": "대각선 접근 후", "align": "전진 전 정렬", "insert": "전진 후"}


class SidePickControl:
    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.cancel = threading.Event()
        self.step = Step.HOME
        self.busy = False
        self.advanced = False
        self.enter_down = False
        self.session = None
        self.ports = None
        self.photos = {}
        self.record = {}
        self.log_lines = []
        root.title("LeArm · 대각선 접근과 파지")
        root.minsize(760, 570)
        root.protocol("WM_DELETE_WINDOW", self.close)

        content = ttk.Frame(root, padding=12)
        content.pack(fill="both", expand=True)
        ttk.Label(content, text="LeArm 대각선 접근과 파지", font=("Malgun Gothic", 16, "bold")).pack(anchor="w")
        self.stage = tk.StringVar()
        self.status = tk.StringVar()
        ttk.Label(content, textvariable=self.stage, font=("Malgun Gothic", 11, "bold")).pack(anchor="w")
        ttk.Label(content, textvariable=self.status, wraplength=850).pack(anchor="w", pady=(2, 8))
        self.motion_status = tk.StringVar(value="대기 중")
        ttk.Label(content, textvariable=self.motion_status).pack(anchor="w")
        self.progress_bar = ttk.Progressbar(content, mode="indeterminate")
        self.progress_bar.pack(fill="x", pady=(3, 10))
        body = ttk.Frame(content)
        body.pack(fill="both", expand=True)
        form = ttk.Frame(body)
        form.pack(side="left", fill="both", expand=True, padx=(0, 10))
        vision = ttk.Frame(body)
        vision.pack(side="right", fill="y")
        photo_area = ttk.Frame(vision, width=320, height=240)
        photo_area.pack()
        photo_area.pack_propagate(False)
        self.preview = ttk.Label(photo_area, text="320 × 240 사진 대기", anchor="center")
        self.preview.pack(fill="both", expand=True)
        self.photo_label = tk.StringVar(value="초기 관찰 자세의 좌표만 이동 목표로 사용")
        ttk.Label(vision, textvariable=self.photo_label, wraplength=320).pack(anchor="w")
        self.photo_choice = tk.StringVar()
        self.photo_history = ttk.Combobox(vision, textvariable=self.photo_choice, state="readonly", width=25)
        self.photo_history.pack(anchor="w")
        self.photo_history.bind("<<ComboboxSelected>>", lambda _event: self.show_photo())
        self.log = tk.Text(vision, width=43, height=9, wrap="word", state="disabled")
        self.log.pack(fill="both", expand=True, pady=(8, 0))

        self.object_name = tk.StringVar(value=OBJECTS["cube"].name)
        self.object_box = ttk.Combobox(form, textvariable=self.object_name, state="readonly",
                                       values=[profile.name for profile in OBJECTS.values()])
        self.object_box.grid(row=0, column=1, sticky="w")
        ttk.Label(form, text="시험 물체 직접 선택").grid(row=0, column=0, sticky="w")
        self.profile_text = tk.StringVar()
        ttk.Label(form, textvariable=self.profile_text, wraplength=470).grid(row=1, columnspan=2, sticky="w")
        self.guide = tk.StringVar()
        ttk.Label(form, textvariable=self.guide, wraplength=390, justify="left").grid(
            row=2, columnspan=2, sticky="w", pady=(12, 6))
        self.settings_button = ttk.Button(form, text="세부 설정 열기", command=self.toggle_settings)
        self.settings_button.grid(row=3, columnspan=2, sticky="w", pady=(8, 4))
        self.settings = ttk.Frame(form)
        self.settings.grid(row=4, columnspan=2, sticky="nw")
        self.settings.grid_remove()
        self.fields = {}
        self.entries = {}
        for row, (key, label, value) in enumerate(FIELDS, start=2):
            ttk.Label(self.settings, text=label).grid(row=row, column=0, sticky="w", padx=(0, 12))
            self.fields[key] = tk.StringVar(value=value)
            self.entries[key] = ttk.Entry(self.settings, textvariable=self.fields[key], width=12)
            self.entries[key].grid(row=row, column=1, sticky="w")
        self.object_box.bind("<<ComboboxSelected>>", self.select_object)
        self.select_object()

        jog_row = ttk.Frame(content)
        jog_row.pack(fill="x", pady=(8, 4))
        self.jog_buttons = []
        for label, vector in (("앞", (1, 0, 0)), ("뒤", (-1, 0, 0)), ("왼쪽", (0, 1, 0)),
                              ("오른쪽", (0, -1, 0)), ("위", (0, 0, 1)), ("아래", (0, 0, -1))):
            button = ttk.Button(jog_row, text=label, width=6, command=lambda v=vector: self.jog(v))
            button.pack(side="left", padx=(0, 3))
            self.jog_buttons.append(button)
        self.photo_button = ttk.Button(jog_row, text="사진 다시 찍기", command=self.retake)
        self.photo_button.pack(side="left")
        self.result = tk.StringVar()
        self.result_box = ttk.Combobox(jog_row, textvariable=self.result, width=14,
                                       values=("들림", "밀림/미파지", "미확인"), state="readonly")
        self.result_box.pack(side="right")

        controls = ttk.Frame(content)
        controls.pack(fill="x")
        ttk.Label(controls, text="단계별 실행").pack(anchor="w")
        step_row = ttk.Frame(controls)
        step_row.pack(fill="x", pady=4)
        self.stage_buttons = {}
        for index, (step, label) in enumerate((
            (Step.HOME, "초기 자세"), (Step.DETECT, "물체 찾기"), (Step.APPROACH, "접근"),
            (Step.LOWER, "하강"), (Step.ALIGN, "자동 보정"), (Step.CLOSE, "닫기"),
            (Step.LIFT, "들어 올리기"), (Step.RELEASE, "내려놓기"),
            (Step.RETURN, "복귀"), (Step.STOPPED, "새 시험 준비"),
        )):
            button = ttk.Button(step_row, text=label, command=self.advance)
            button.grid(row=index // 5, column=index % 5, padx=(0, 4), pady=2, sticky="ew")
            self.stage_buttons[step] = button
        ttk.Button(controls, text="시험 기록 저장", command=self.save_record).pack(side="left", padx=5)
        self.stop_button = ttk.Button(controls, text="다음 명령 중단", command=self.stop)
        self.stop_button.pack(side="right")
        automatic = ttk.Frame(content)
        automatic.pack(fill="x", pady=(6, 0))
        self.auto_ready = tk.BooleanVar()
        self.auto_check = ttk.Checkbutton(automatic, variable=self.auto_ready,
                                          text="작업면 배치 유지 · 빈 집게 · 손과 전체 경로 비움")
        self.auto_check.pack(side="left")
        self.auto_button = ttk.Button(automatic, text="전체 자동 실행", command=self.automatic_pick)
        self.auto_button.pack(side="right")
        self.background_button = ttk.Button(automatic, text="빈 배경 촬영", command=self.capture_background)
        self.background_button.pack(side="right", padx=(0, 6))
        root.bind("<KeyPress-Return>", self.press_enter)
        root.bind("<KeyRelease-Return>", self.release_enter)
        self.show_step()
        root.after(50, self.refresh)

    @property
    def profile(self):
        return next(profile for profile in OBJECTS.values() if profile.name == self.object_name.get())

    def select_object(self, _event=None):
        profile = self.profile
        self.profile_text.set(f"중간 높이 {profile.grasp_height_mm:g} mm · 파지 폭 {profile.grip_width_mm} mm\n"
                              f"{profile.alignment}")
        self.fields["close"].set("425" if profile is OBJECTS["cube"] else str(GRASP_PRESETS[profile][-1]))
        self.fields["pitch"].set("-60" if profile is OBJECTS["cube"] else "-55")
        self.guide.set("1. 작업면을 비우고 확인란에 체크한 뒤 빈 배경 촬영을 눌러요.\n"
                       "2. 물체 하나를 놓고 같은 물체를 화면에서 선택해요.\n"
                       "3. 손과 경로를 비운 뒤 체크하고 전체 자동 실행을 눌러요. 집고 내려놓은 뒤 초기 자세로 돌아와요.\n"
                       "한 동작씩 확인하려면 아래 단계별 버튼을 순서대로 눌러요.")
        for key in ("height", "opening", "insert", "offset_x", "offset_y"):
            self.fields[key].set("")

    def press_enter(self, _event):
        inputs = (*self.entries.values(), self.object_box, self.result_box, self.photo_history, self.auto_button)
        if self.root.focus_get() not in inputs and not self.enter_down:
            self.enter_down = True
            self.advance()
        return "break"

    def release_enter(self, _event):
        self.enter_down = False

    def show_step(self):
        label, instruction = STAGES[self.step]
        if self.step is Step.ALIGN and not self.advanced:
            label = "사진으로 좌우·앞뒤 보정"
            instruction = "자동 보정을 누르면 사진을 다시 찍으며 위치를 맞춰요. 다음 단계에서 닫기를 누르세요."
        self.stage.set(label)
        self.status.set(instruction)
        if not self.busy:
            self.motion_status.set("중단됨" if self.step is Step.STOPPED else "대기 중 · 다음 조작을 선택하세요")
        self.settings_button.configure(state="disabled" if self.busy else "normal")
        for step, button in self.stage_buttons.items():
            button.configure(state="normal" if self.step is step and not self.busy else "disabled")
        align = self.step in (Step.LOWER, Step.ALIGN) and not self.busy
        for index, button in enumerate(self.jog_buttons):
            enabled = align and (self.step is Step.ALIGN or index < 4)
            button.configure(state="normal" if enabled else "disabled")
        self.photo_button.configure(state="normal" if align else "disabled")
        unlocked = self.step is Step.HOME and not self.busy
        self.object_box.configure(state="readonly" if unlocked else "disabled")
        for key, entry in self.entries.items():
            entry.configure(state="normal" if not self.busy and (unlocked or key not in ("camera", "arm"))
                            else "disabled")
        self.result_box.configure(state="readonly" if self.step is Step.RELEASE and not self.busy else "disabled")
        self.stop_button.configure(state="normal" if self.step is not Step.STOPPED else "disabled")
        auto = self.step in (Step.HOME, Step.DETECT) and not self.busy
        for widget in (self.auto_check, self.auto_button, self.background_button):
            widget.configure(state="normal" if auto else "disabled")

    def capture_background(self):
        if self.busy or self.step not in (Step.HOME, Step.DETECT):
            return
        if not self.auto_ready.get():
            self.status.set("작업면과 집게를 비우고, 손과 복귀 경로를 확인한 뒤 체크하세요.")
            return
        self.auto_ready.set(False)
        try:
            if self.step is Step.HOME:
                self.begin("connect_and_capture")
            else:
                self.start("capture_background")
        except ValueError as error:
            self.status.set(str(error))

    def toggle_settings(self):
        self.advanced = not self.advanced
        if self.advanced:
            self.settings.grid()
        else:
            self.settings.grid_remove()
        self.settings_button.configure(text="세부 설정 닫기" if self.advanced else "세부 설정 열기")
        self.show_step()

    def automatic_pick(self):
        if self.busy or self.step not in (Step.HOME, Step.DETECT):
            return
        if not self.auto_ready.get():
            self.status.set("작업면 배치와 빈 경로를 확인한 뒤 자동 집기를 누르세요.")
            return
        self.auto_ready.set(False)
        try:
            if self.step is Step.HOME:
                self.begin("connect_and_run")
            else:
                self.start("demo_run")
        except ValueError as error:
            self.status.set(str(error))

    def read(self, name, kind=int):
        try:
            return kind(self.fields[name].get())
        except ValueError:
            label = next(label for key, label, _ in FIELDS if key == name)
            raise ValueError(f"{label} 값을 입력하세요.") from None

    def begin(self, action):
        ports = tuple(self.fields[key].get().strip() for key in ("camera", "arm"))
        if not all(ports) or ports[0].casefold() == ports[1].casefold():
            raise ValueError("카메라와 로봇팔의 서로 다른 포트를 입력하세요.")
        self.record = {"object": self.profile.name, "nominal_grasp_height_mm": self.profile.grasp_height_mm}
        self.photos = {}
        self.photo_choice.set("")
        self.photo_history.configure(values=())
        self.log_lines = []
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")
        self.preview.configure(image="", text="320 × 240 사진 대기")
        self.photo_label.set("초기 관찰 자세의 좌표만 이동 목표로 사용")
        self.result.set("")
        for key in ("height", "opening", "insert", "offset_x", "offset_y"):
            self.fields[key].set("")
        self.start(action, self.profile, ports)

    def advance(self):
        if self.busy:
            return
        try:
            if self.step is Step.STOPPED:
                self.disconnect()
                self.cancel.clear()
                self.step = Step.HOME
                self.show_step()
                return
            if self.step is Step.HOME:
                self.begin("connect")
            elif self.step is Step.DETECT:
                offsets = (self.read("forward"), self.read("left")) if self.advanced else (0, 0)
                self.start("detect", *offsets)
            elif self.step is Step.APPROACH:
                if self.advanced:
                    self.start("approach", self.read("standoff"), self.read("rise"))
                else:
                    self.start("demo_approach")
            elif self.step in (Step.LOWER, Step.ALIGN):
                if not self.photos or self.photo_choice.get() != next(reversed(self.photos)):
                    raise ValueError("가장 최근 사진을 선택하세요.")
                if self.step is Step.LOWER:
                    if self.advanced:
                        self.start("lower", self.read("height", float), self.read("height_trim", float),
                                   self.read("opening", float), self.read("pitch", float), self.read("roll"))
                    else:
                        self.start("demo_lower")
                elif self.advanced:
                    self.start("insert", self.read("insert"))
                else:
                    self.start("demo_align")
            elif self.step is Step.CLOSE:
                if self.advanced:
                    self.start("close", self.read("close"))
                else:
                    self.start("demo_close")
            elif self.step is Step.LIFT:
                self.start("lift")
            elif self.step is Step.RELEASE:
                if not self.result.get():
                    raise ValueError("들렸는지 시험 결과를 먼저 선택하세요.")
                offsets = {key: self.read(key, float) for key in ("offset_x", "offset_y") if self.fields[key].get()}
                self.record.update(result=self.result.get(), measured_offsets_mm=offsets)
                self.start("release")
            else:
                self.start("finish")
        except ValueError as error:
            self.status.set(str(error))

    def jog(self, vector):
        if self.busy or self.step not in (Step.LOWER, Step.ALIGN):
            return
        try:
            amount = self.read("jog")
            if not 1 <= amount <= 10:
                raise ValueError("미세 이동은 1~10 mm로 입력하세요.")
            self.start("jog", *(component * amount for component in vector))
        except ValueError as error:
            self.status.set(str(error))

    def retake(self):
        if not self.busy and self.step in (Step.LOWER, Step.ALIGN):
            self.start("snapshot", "approach" if self.step is Step.LOWER else "align")

    def start(self, action, *args):
        if action != "connect":
            self.record.setdefault("requested_steps", []).append({"action": action, "parameters": args})
        self.busy = True
        self.show_step()
        self.status.set("실행 중 · 다음 단계는 동작이 끝난 뒤 선택하세요.")
        self.motion_status.set("진행 중 · 명령 준비")
        self.progress_bar.start(12)
        threading.Thread(target=self.run, args=(action, args), daemon=True).start()

    def connect(self, profile, ports, *, require_background=False):
        import serial

        camera = serial.Serial(baudrate=115200, timeout=.12, write_timeout=2)
        camera.port = ports[0]
        camera.dtr = camera.rts = False
        self.events.put(("progress", f"카메라 {ports[0]} 연결 확인"))
        try:
            camera.open()
        except serial.SerialException as error:
            raise OSError(f"카메라 {ports[0]}을 열 수 없어요. CanMV IDE나 이전 조작창이 연결 중이면 닫으세요. ({error})") from error
        stream = CameraStream(camera)
        try:
            stream.wait_ready(self.check_cancelled, require_background=require_background)
            self.events.put(("progress", f"로봇팔 {ports[1]} 연결"))
            arm = LeArmSerial(ports[1])
        except Exception:
            camera.close()
            raise
        self.ports = (camera, arm)
        self.session = SidePick(stream, arm, profile, report=lambda m: self.events.put(("log", m)),
                                on_photo=lambda label, jpeg: self.events.put(("photo", (label, jpeg))),
                                cancelled=self.cancel.is_set,
                                progress=lambda message: self.events.put(("progress", message)))
        try:
            self.session.home()
        except Exception:
            self.disconnect()
            raise

    def check_cancelled(self):
        if self.cancel.is_set():
            raise InterruptedError("중단 요청을 받았어요.")

    def run(self, action, args):
        automatic = action in ("connect_and_run", "demo_run")
        try:
            if action in ("connect", "connect_and_run", "connect_and_capture"):
                self.connect(*args, require_background=action == "connect_and_capture")
            if automatic:
                run_demo(self.session)
                self.disconnect()
            elif action in ("connect_and_capture", "capture_background"):
                self.session.capture_background()
                self.disconnect()
            elif action == "demo_approach":
                demo_approach(self.session)
            elif action == "demo_lower":
                demo_lower(self.session)
            elif action == "demo_align":
                jpeg = self.session.snapshot("align")
                align_demo_depth(self.session, jpeg)
                self.session.insert(0)
            elif action == "demo_close":
                close_demo_object(self.session)
            elif action == "finish":
                self.session.home()
                self.disconnect()
            elif action != "connect":
                getattr(self.session, action)(*args)
            step = self.session.step if self.session is not None else Step.HOME
            self.events.put(("done", (step, None)))
            if automatic:
                self.events.put(("completed", "자동 시연 완료 · 내려놓기 및 초기 복귀"))
            elif action in ("connect_and_capture", "capture_background"):
                self.events.put(("background_ready", "배경 준비 완료 · 물체 하나를 놓으세요."))
        except (ValueError, TimeoutError) as error:
            step = self.session.step if self.session is not None else Step.HOME
            if step is not Step.DETECT and (automatic or action == "demo_close" or isinstance(error, TimeoutError)):
                step = Step.STOPPED
            self.events.put(("done", (step, str(error))))
        except Exception as error:
            step = Step.HOME if self.session is None and not self.cancel.is_set() else Step.STOPPED
            self.events.put(("done", (step, str(error))))

    def refresh(self):
        while True:
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "done":
                self.step, error = value
                if self.cancel.is_set():
                    self.step = Step.STOPPED
                self.busy = False
                self.progress_bar.stop()
                self.progress_bar.configure(value=0)
                self.show_step()
                if error:
                    self.status.set(error)
                    self.append_log(error)
            elif kind == "background_ready":
                if not self.cancel.is_set():
                    self.status.set(value)
                    self.append_log(value)
            elif kind == "completed" and not self.cancel.is_set():
                self.status.set(value)
                self.record["result"] = "자동 명령 완료 · 실제 파지 결과는 육안 확인"
                self.append_log(value)
            elif kind == "completed":
                continue
            elif kind == "log":
                self.append_log(value)
            elif kind == "progress":
                if self.busy and not self.cancel.is_set():
                    self.motion_status.set(f"진행 중 · {value}")
            else:
                label, jpeg = value
                key = f"{len(self.photos) + 1:02d}-{label}"
                self.photos[key] = jpeg
                self.photo_history.configure(values=tuple(self.photos))
                self.photo_choice.set(key)
                self.show_photo()
        self.root.after(50, self.refresh)

    def show_photo(self):
        key = self.photo_choice.get()
        with Image.open(BytesIO(self.photos[key])) as image:
            self.current_photo = ImageTk.PhotoImage(image)
        self.preview.configure(image=self.current_photo, text="")
        self.photo_label.set(PHOTOS[key.split("-", 1)[1]] + " · 원본 사진")

    def append_log(self, message):
        self.log_lines.append(message)
        self.log.configure(state="normal")
        self.log.insert("end", message + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def save_record(self):
        if self.busy or not self.record:
            return
        filename = filedialog.asksaveasfilename(title="파지 시험 기록 저장", defaultextension=".json",
                                                filetypes=[("JSON", "*.json")])
        if not filename:
            return
        path = Path(filename)
        try:
            photos = {}
            for label, jpeg in self.photos.items():
                photo_path = path.with_name(f"{path.stem}-{label}.jpg")
                photo_path.write_bytes(jpeg)
                photos[label] = photo_path.name
            path.write_text(json.dumps({**self.record, "photos": photos, "log": self.log_lines},
                                       ensure_ascii=False, indent=2), encoding="utf-8")
            self.status.set(f"시험 기록 저장 완료 · {path.name}")
        except OSError as error:
            self.status.set(str(error))

    def stop(self):
        self.cancel.set()
        if not self.busy:
            self.step = Step.STOPPED
            self.show_step()
        self.status.set("이후 명령 전송 중단 요청 · 이미 전송한 동작은 끝까지 실행될 수 있어요.")
        if self.busy:
            self.motion_status.set("중단 요청 · 현재 동작 확인 중")

    def disconnect(self):
        if self.ports is not None:
            camera, arm = self.ports
            try:
                arm.close()
            finally:
                camera.close()
        self.ports = None
        self.session = None

    def close(self):
        if self.busy:
            self.stop()
            return
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    window = tk.Tk()
    SidePickControl(window)
    window.mainloop()
