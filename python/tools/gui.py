import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from agibot_hand import AgibotHandO12, EFinger, EControlMode, EHandType
from collections import deque
import threading
import time
import math

class OmniHandGUI:
    def __init__(self):
        self.hand = AgibotHandO12(hand_type=EHandType.RIGHT)
        self.window_size = 100
        self.sample_count = 0
        self.running = True
        self.control_mode = "position"  # "position" or "angle"
        self.show_rad = True  # 默认显示弧度制
        
        # 数据缓冲区
        self.times = deque(maxlen=self.window_size)
        self.forces = {name: deque(maxlen=self.window_size) 
                      for name in ['Thumb', 'Index', 'Middle', 'Ring', 'Little']}
        self.all_values = deque(maxlen=200)
        
        # 存储当前角度值
        self.current_angles = [0.0] * 12
        
        # 关节角度范围（弧度）
        self.angle_ranges = [
            (0, 0.94),      # 拇指Roll
            (-1.39, 0),     # 拇指AbAd
            (-0.83, 0),     # 拇指MCP
            (-1.29, 0),     # 拇指PIP
            (-0.26, 0.26),  # 食指AbAd
            (0, 1.35),      # 食指MCP
            (0, 1.53),      # 食指PIP
            (-0.26, 0.26),  # 中指AbAd
            (0, 1.36),      # 中指MCP
            (0, 1.82),      # 中指PIP
            (0, 1.54),      # 无名指MCP
            (0, 1.54)       # 小指MCP
        ]
        
        # 关节名称
        self.joint_names = [
            "拇指Roll", "拇指AbAd", "拇指MCP", "拇指PIP",
            "食指AbAd", "食指MCP", "食指PIP",
            "中指AbAd", "中指MCP", "中指PIP",
            "无名指MCP", "小指MCP"
        ]
        
        self.setup_ui()
        self.init_hand()
        self.start_monitoring()
    def setup_ui(self):
        # 主窗口
        self.root = tk.Tk()
        self.root.title("OmniHand Pro 2025 - Joint Control System")
        self.root.geometry("1600x820")
        self.root.configure(bg='#000000')
        
        main_frame = tk.Frame(self.root, bg='#000000', relief='flat', bd=0)
        main_frame.pack(fill='both', expand=True, padx=12, pady=12)
        
        # 左侧面板
        self.setup_left_panel(main_frame)
        
        # 右侧图表
        self.setup_chart_panel(main_frame)
        
    def setup_left_panel(self, parent):
        left_frame = tk.Frame(parent, bg='#0d0d0d', width=500, relief='flat', bd=0)
        left_frame.pack(side='left', fill='y', padx=(0, 12))
        left_frame.pack_propagate(False)
        
        tech_line = tk.Frame(left_frame, bg='#00d4ff', height=1)
        tech_line.pack(fill='x', padx=0, pady=(0, 0))
        
        self.setup_mode_switch(left_frame)
        self.setup_joint_control(left_frame)
        
    def setup_mode_switch(self, parent):
        mode_frame = tk.Frame(parent, bg='#0d0d0d', height=60)
        mode_frame.pack(fill='x', padx=20, pady=(18, 15))
        mode_frame.pack_propagate(False)
        
        # 控制模式标题和按钮
        control_frame = tk.Frame(mode_frame, bg='#0d0d0d')
        control_frame.pack(fill='x', pady=(0, 8))
        
        tk.Label(control_frame, text="控制模式", 
                bg='#0d0d0d', fg='#00d4ff', 
                font=('Consolas', 13, 'bold')).pack(anchor='w')
        
        switch_frame = tk.Frame(control_frame, bg='#0d0d0d')
        switch_frame.pack(fill='x', pady=(8, 0))
        
        self.position_btn = tk.Button(switch_frame, text="位置控制", 
                                    command=lambda: self.switch_mode("position"),
                                    bg='#00d4ff', fg='#000000',
                                    font=('Consolas', 9, 'bold'),
                                    relief='flat', bd=0, width=10, height=2)
        self.position_btn.pack(side='left', padx=(0, 5))
        
        self.angle_btn = tk.Button(switch_frame, text="角度控制", 
                                 command=lambda: self.switch_mode("angle"),
                                 bg='#1a1a1a', fg='#cccccc',
                                 font=('Consolas', 9, 'bold'),
                                 relief='flat', bd=0, width=10, height=2)
        self.angle_btn.pack(side='left')
        
        # 显示单位切换
        unit_frame = tk.Frame(mode_frame, bg='#0d0d0d')
        unit_frame.pack(fill='x', pady=(8, 0))
        
        tk.Label(unit_frame, text="显示单位", 
                bg='#0d0d0d', fg='#00d4ff', 
                font=('Consolas', 13, 'bold')).pack(anchor='w')
        
        unit_switch_frame = tk.Frame(unit_frame, bg='#0d0d0d')
        unit_switch_frame.pack(fill='x', pady=(8, 0))
        
        self.rad_btn = tk.Button(unit_switch_frame, text="弧度制", 
                               command=lambda: self.switch_unit(True),
                               bg='#00d4ff', fg='#000000',
                               font=('Consolas', 9, 'bold'),
                               relief='flat', bd=0, width=10, height=2)
        self.rad_btn.pack(side='left', padx=(0, 5))
        
        self.deg_btn = tk.Button(unit_switch_frame, text="角度制", 
                               command=lambda: self.switch_unit(False),
                               bg='#1a1a1a', fg='#cccccc',
                               font=('Consolas', 9, 'bold'),
                               relief='flat', bd=0, width=10, height=2)
        self.deg_btn.pack(side='left')
        
        separator = tk.Frame(parent, bg='#222222', height=1)
        separator.pack(fill='x', padx=20, pady=10)
        
    def setup_joint_control(self, parent):
        title_frame = tk.Frame(parent, bg='#0d0d0d', height=40)
        title_frame.pack(fill='x', padx=20, pady=(0, 8))
        title_frame.pack_propagate(False)
        
        self.control_title = tk.Label(title_frame, text="关节位置控制", 
                                    bg='#0d0d0d', fg='#00d4ff', 
                                    font=('Consolas', 13, 'bold'))
        self.control_title.pack(anchor='w')
        
        control_container = tk.Frame(parent, bg='#0d0d0d')
        control_container.pack(fill='both', expand=True, padx=20)
        
        self.position_panel = tk.Frame(control_container, bg='#0d0d0d')
        self.setup_position_panel(self.position_panel)
        
        self.angle_panel = tk.Frame(control_container, bg='#0d0d0d')
        self.setup_angle_panel(self.angle_panel)
        
        self.position_panel.pack(fill='both', expand=True)
    def setup_position_panel(self, parent):
        joints_frame = tk.Frame(parent, bg='#0d0d0d')
        joints_frame.pack(fill='both', expand=True)
        
        self.position_sliders = []
        
        for i, name in enumerate(self.joint_names):
            joint_frame = tk.Frame(joints_frame, bg='#151515', relief='flat', bd=0)
            joint_frame.pack(fill='x', pady=2)
            
            info_frame = tk.Frame(joint_frame, bg='#151515')
            info_frame.pack(fill='x', padx=12, pady=5)
            
            joint_label = tk.Label(info_frame, text="ID " + str(i + 1), 
                                 bg='#151515', fg='#cccccc', 
                                 font=('Consolas', 9, 'bold'), width=8)
            joint_label.pack(side='left')
            
            slider = tk.Scale(info_frame, from_=0, to=2000, orient='horizontal',
                            bg='#151515', fg='#888888', 
                            troughcolor='#2a2a2a', activebackground='#00d4ff',
                            highlightthickness=0, bd=0, length=170, showvalue=0,
                            command=lambda v, idx=i: self.on_position_slider_change(idx, v))
            slider.set(2000 if i < 11 else 0)
            slider.pack(side='left', padx=(10, 15))
            
            value_label = tk.Label(info_frame, text=str(2000 if i < 11 else 0), 
                                 bg='#151515', fg='#00d4ff', 
                                 font=('Consolas', 9, 'bold'), width=4)
            value_label.pack(side='right')
            
            self.position_sliders.append((slider, value_label))
            
    def setup_angle_panel(self, parent):
        joints_frame = tk.Frame(parent, bg='#0d0d0d')
        joints_frame.pack(fill='both', expand=True)
        
        self.angle_sliders = []
        
        for i, (name, (min_angle, max_angle)) in enumerate(zip(self.joint_names, self.angle_ranges)):
            joint_frame = tk.Frame(joints_frame, bg='#151515', relief='flat', bd=0)
            joint_frame.pack(fill='x', pady=2)
            
            info_frame = tk.Frame(joint_frame, bg='#151515')
            info_frame.pack(fill='x', padx=12, pady=5)
            
            # 添加范围显示
            range_text = f"[{min_angle:.2f}, {max_angle:.2f}]" if self.show_rad else f"[{math.degrees(min_angle):.1f}°, {math.degrees(max_angle):.1f}°]"
            joint_label = tk.Label(info_frame, 
                                 text=f"{name}\n{range_text}", 
                                 bg='#151515', fg='#cccccc', 
                                 font=('Consolas', 9, 'bold'), 
                                 width=16,
                                 justify='left')
            joint_label.pack(side='left')
            
            slider = tk.Scale(info_frame, from_=min_angle, to=max_angle, 
                            orient='horizontal', resolution=0.01,
                            bg='#151515', fg='#888888', 
                            troughcolor='#2a2a2a', activebackground='#00d4ff',
                            highlightthickness=0, bd=0, length=170, showvalue=0,
                            command=lambda v, idx=i: self.on_angle_slider_change(idx, v))
            slider.set(self.current_angles[i])
            slider.pack(side='left', padx=(10, 15))
            
            value_label = tk.Label(info_frame, 
                                 text=self.format_angle_value(self.current_angles[i]), 
                                 bg='#151515', fg='#00d4ff', 
                                 font=('Consolas', 9, 'bold'), width=8)
            value_label.pack(side='right')
            
            self.angle_sliders.append((slider, value_label))

    def format_angle_value(self, angle_rad):
        """格式化角度值显示"""
        if self.show_rad:
            return f"{angle_rad:.3f}"
        else:
            return f"{math.degrees(angle_rad):.1f}°"

    def on_position_slider_change(self, joint_idx, value):
        """位置滑动条变化回调"""
        value = int(float(value))
        self.position_sliders[joint_idx][1].config(text=str(value))
        try:
            self.hand.set_joint_position(joint_idx + 1, value)
        except Exception as e:
            print(f"设置关节位置失败: {e}")

    def on_angle_slider_change(self, joint_idx, value):
        """角度滑动条变化回调"""
        angle_rad = float(value)
        # 更新显示
        self.update_angle_display(joint_idx, angle_rad)
        
        try:
            # 更新存储的角度值
            self.current_angles[joint_idx] = angle_rad
            # 一次性设置所有关节角度
            self.hand.set_all_active_joint_angles(self.current_angles)
        except Exception as e:
            print(f"设置关节角度失败: {e}")

    def update_angle_display(self, joint_idx, angle_rad):
        """更新角度显示"""
        self.angle_sliders[joint_idx][1].config(
            text=self.format_angle_value(angle_rad)
        )

    def switch_unit(self, show_rad):
        """切换显示单位（弧度/角度）"""
        self.show_rad = show_rad
        if show_rad:
            self.rad_btn.config(bg='#00d4ff', fg='#000000')
            self.deg_btn.config(bg='#1a1a1a', fg='#cccccc')
        else:
            self.rad_btn.config(bg='#1a1a1a', fg='#cccccc')
            self.deg_btn.config(bg='#00d4ff', fg='#000000')
        
        # 更新所有角度显示
        if hasattr(self, 'angle_sliders'):
            for i, (slider, _) in enumerate(self.angle_sliders):
                self.update_angle_display(i, float(slider.get()))
    def setup_chart_panel(self, parent):
        chart_frame = tk.Frame(parent, bg='#000000', relief='flat', bd=0)
        chart_frame.pack(side='right', fill='both', expand=True)
        
        tech_line = tk.Frame(chart_frame, bg='#00d4ff', height=1)
        tech_line.pack(fill='x', padx=0, pady=(0, 0))
        
        title_frame = tk.Frame(chart_frame, bg='#000000', height=60)
        title_frame.pack(fill='x', padx=25, pady=(18, 0))
        title_frame.pack_propagate(False)
        
        title_label = tk.Label(title_frame, text="触觉传感检测", 
                             bg='#000000', fg='#00d4ff', 
                             font=('Consolas', 15, 'bold'))
        title_label.pack(side='left', anchor='w')
        
        status_frame = tk.Frame(title_frame, bg='#000000')
        status_frame.pack(side='right', anchor='e')
        
        self.status_indicator = tk.Label(status_frame, text="●", 
                                      bg='#000000', fg='#00ff88', 
                                      font=('Consolas', 14))
        self.status_indicator.pack(side='left')
        
        tk.Label(status_frame, text="ACTIVE", 
                bg='#000000', fg='#00ff88', 
                font=('Consolas', 9, 'bold')).pack(side='left', padx=(5, 0))
        
        self.setup_matplotlib_chart(chart_frame)

    def setup_matplotlib_chart(self, parent):
        plt.style.use('dark_background')
        self.figure = Figure(figsize=(13, 8), facecolor='#000000')
        self.ax = self.figure.add_subplot(111, facecolor='#080808')
        
        self.ax.set_xlabel('Time (Sample Points)', color='#888888', fontsize=10)
        self.ax.set_ylabel('Normal Force (N)', color='#888888', fontsize=10)
        self.ax.tick_params(colors='#666666', labelsize=8)
        
        self.ax.grid(True, alpha=0.15, color='#333333')
        
        self.ax.set_xlim(0, self.window_size)
        self.ax.set_ylim(0, 50)
        
        colors = {'Thumb': '#00d4ff', 'Index': '#00ff88', 'Middle': '#ff6b35', 
                 'Ring': '#ffd700', 'Little': '#ff69b4'}
        
        self.lines = {}
        for name, color in colors.items():
            line, = self.ax.plot([], [], color=color, linewidth=2, label=name)
            self.lines[name] = line
        
        self.ax.legend(loc='upper right')
        self.figure.tight_layout()
        
        self.canvas = FigureCanvasTkAgg(self.figure, parent)
        self.canvas.get_tk_widget().pack(fill='both', expand=True, padx=25, pady=25)

    def switch_mode(self, mode):
        """切换控制模式"""
        self.control_mode = mode
        if mode == "position":
            self.control_title.config(text="关节位置控制")
            self.position_btn.config(bg='#00d4ff', fg='#000000')
            self.angle_btn.config(bg='#1a1a1a', fg='#cccccc')
            self.position_panel.pack(fill='both', expand=True)
            self.angle_panel.pack_forget()
        else:
            self.control_title.config(text="关节角度控制")
            self.position_btn.config(bg='#1a1a1a', fg='#cccccc')
            self.angle_btn.config(bg='#00d4ff', fg='#000000')
            self.angle_panel.pack(fill='both', expand=True)
            self.position_panel.pack_forget()
            
            # 更新所有滑动条到当前角度值
            for i, (slider, _) in enumerate(self.angle_sliders):
                slider.set(self.current_angles[i])

    def init_hand(self):
        """初始化机械手"""
        try:
            # 位置控制初始化
            init_positions = [2000] * 11 + [0]
            self.hand.set_all_joint_positions(init_positions)
            self.update_slider_display(init_positions)
            
            # 角度控制初始化
            self.current_angles = [0.0] * 12
            self.hand.set_all_active_joint_angles(self.current_angles)
        except Exception as e:
            print(f"初始化失败: {e}")

    def update_slider_display(self, positions):
        """更新位置滑动条显示"""
        for i, (slider, label) in enumerate(self.position_sliders):
            if i < len(positions):
                slider.set(positions[i])
                label.config(text=str(positions[i]))

    def start_monitoring(self):
        """启动监控线程"""
        monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        monitor_thread.start()

    def monitor_loop(self):
        """监控循环"""
        finger_map = {
            'Thumb': EFinger.THUMB,
            'Index': EFinger.INDEX,
            'Middle': EFinger.MIDDLE,
            'Ring': EFinger.RING,
            'Little': EFinger.LITTLE
        }
        
        while self.running:
            try:
                self.times.append(self.sample_count)
                
                for name in self.forces.keys():
                    try:
                        force = max(0, self.hand.get_touch_sensor_data(finger_map[name]).normal_force)
                    except:
                        force = 0
                    
                    self.forces[name].append(force)
                    self.all_values.append(force)
                    self.lines[name].set_data(list(self.times), list(self.forces[name]))
                
                self.update_display_range()
                self.root.after(0, self.canvas.draw)
                
                self.sample_count += 1
                time.sleep(0.05)
                
            except Exception as e:
                print(f"监控错误: {e}")
                break

    def update_display_range(self):
        """更新显示范围"""
        if len(self.times) > 0:
            latest_time = self.times[-1]
            if latest_time > self.window_size:
                self.ax.set_xlim(latest_time - self.window_size, latest_time)
        
        if self.sample_count % 20 == 0 and len(self.all_values) > 5:
            y_min = max(0, min(self.all_values))
            y_max = max(self.all_values)
            
            if y_max - y_min < 50:
                y_center = (y_max + y_min) / 2
                y_min = max(0, y_center - 25)
                y_max = y_min + 50
            else:
                margin = (y_max - y_min) * 0.2
                y_max = y_max + margin
            
            self.ax.set_ylim(y_min, y_max)

    def on_closing(self):
        """关闭窗口回调"""
        self.running = False
        self.root.destroy()

    def run(self):
        """运行GUI"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

def main():
    gui = OmniHandGUI()
    gui.run()

if __name__ == '__main__':
    main()