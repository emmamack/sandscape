import tkinter as tk
from tkinter import font
import threading
import queue
import difflib
import time
from collections import OrderedDict

class DebuggerWindow:
    """
    A persistent GUI window for displaying and debugging values from a fast-moving control loop.
    It highlights changes between frames and keeps all variables visible, marking stale data.
    """

    def __init__(self):
        """Initializes the debugger state without creating any GUI components yet."""
        self._command_queue = queue.Queue()
        self._gui_thread = None
        self.is_active = False
        
        # Data storage
        self._frames = []  # List of historical frames (dictionaries)
        self._current_frame_data = OrderedDict() # Data for the frame currently being built
        self._all_known_ids = OrderedDict() # Tracks all IDs ever seen to maintain order
        
        # State for viewing historical data
        self._view_mode = 'live'  # 'live' or 'paused'
        self._view_frame_index = -1
        
        # For handling debounced UI updates
        self._recalc_job = None

    def startup(self):
        """
        Creates and starts the GUI in a separate thread.
        This method is safe to call multiple times.
        """
        if self.is_active:
            return

        self.is_active = True
        self._gui_thread = threading.Thread(target=self._run_gui, daemon=True)
        self._gui_thread.start()

    def show(self, id_str: str, value: object):
        """
        Displays a value in the debugger window for the current frame.
        
        Args:
            id_str: A unique string identifier for the value.
            value: The value to display. Its __repr__ will be used.
        """
        if not self.is_active:
            return
        self._command_queue.put(('show', id_str, repr(value)))

    def next(self):
        """
        Signals the end of the current frame and the beginning of a new one.
        This saves the current state and prepares for new data.
        """
        if not self.is_active:
            return
        self._command_queue.put(('next',))
        
    def close(self):
        """Stops the GUI thread and closes the window."""
        if not self.is_active:
            return
        self._command_queue.put(('close',))


    def _run_gui(self):
        """The main entry point for the GUI thread."""
        self.root = tk.Tk()
        self.root.title("Debugger Window")
        self.root.geometry("800x600")
        
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

        # --- Configure Fonts and Colors ---
        self._default_font = font.nametofont("TkDefaultFont")
        self._monospace_font = font.Font(family="Courier", size=10)
        self._colors = {
            'bg': '#2E2E2E',
            'fg': '#DCDCDC',
            'label_bg': '#3C3C3C',
            'widget_bg': '#1E1E1E',
            'stale_fg': '#777777',
            'highlight_add': '#2A512A',
            'highlight_change': '#644D21',
            'button_bg': '#4A4A4A',
            'button_fg': '#FFFFFF'
        }

        self.root.configure(bg=self._colors['bg'])

        # --- GUI Layout ---
        control_frame = tk.Frame(self.root, bg=self._colors['bg'])
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        self.main_canvas = tk.Canvas(self.root, bg=self._colors['widget_bg'], highlightthickness=0)
        scrollbar = tk.Scrollbar(self.root, orient="vertical", command=self.main_canvas.yview)
        self._scrollable_frame = tk.Frame(self.main_canvas, bg=self._colors['widget_bg'])

        self._scrollable_frame.bind(
            "<Configure>",
            lambda e: self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all"))
        )
        
        self.canvas_window = self.main_canvas.create_window((0, 0), window=self._scrollable_frame, anchor="nw")
        self.main_canvas.configure(yscrollcommand=scrollbar.set)
        
        self.main_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5,0), pady=(0,5))
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, padx=(0,5), pady=(0,5))
        
        self.main_canvas.bind('<Configure>', self._on_canvas_resize)

        # --- Control Widgets ---
        self._pause_button = tk.Button(control_frame, text="Pause Live Display", command=self._toggle_pause, bg=self._colors['button_bg'], fg=self._colors['button_fg'], width=20)
        self._prev_button = tk.Button(control_frame, text="< Previous Frame", command=self._prev_frame, state=tk.DISABLED, bg=self._colors['button_bg'], fg=self._colors['button_fg'])
        self._next_button = tk.Button(control_frame, text="Next Frame >", command=self._next_frame, state=tk.DISABLED, bg=self._colors['button_bg'], fg=self._colors['button_fg'])
        self._frame_info_label = tk.Label(control_frame, text="Display: Live", bg=self._colors['bg'], fg=self._colors['fg'], width=30)

        self._pause_button.pack(side=tk.LEFT, padx=5)
        self._prev_button.pack(side=tk.LEFT, padx=5)
        self._next_button.pack(side=tk.LEFT, padx=5)
        self._frame_info_label.pack(side=tk.LEFT, padx=10)

        self._display_widgets = {}

        self.root.after(100, self._process_queue)
        self.root.mainloop()

    def _on_canvas_resize(self, event):
        """When canvas is resized, update the width of the frame inside and schedule a height recalc."""
        canvas_width = event.width
        self.main_canvas.itemconfig(self.canvas_window, width=canvas_width)
        self._schedule_recalc()

    def _schedule_recalc(self):
        """Schedules a debounced call to recalculate widget heights."""
        if self._recalc_job:
            self.root.after_cancel(self._recalc_job)
        # A short delay is good for debouncing resize events
        self._recalc_job = self.root.after(50, self._recalculate_all_heights)

    def _on_closing(self):
        """Handle the window close event."""
        self.is_active = False
        self.root.destroy()

    def _recalculate_all_heights(self):
        """Iterates through all visible widgets and adjusts their height based on wrapping."""
        self._recalc_job = None
        try:
            # KEY FIX: Force tkinter to process all pending layout changes NOW.
            # This ensures widget widths are correct before we query them for their display lines.
            self.root.update_idletasks()
        except tk.TclError:
            return # Window was closed, so abort.

        for _, value_text in self._display_widgets.values():
            try:
                value_text.configure(state=tk.NORMAL)
                display_lines = value_text.count('1.0', 'end', 'displaylines')
                print("display_lines=",display_lines)
                value_text.configure(height=max(1, display_lines))
                value_text.configure(state=tk.DISABLED)
            except tk.TclError:
                pass # Widget might have been destroyed

    def _process_queue(self):
        """Processes commands from the main thread's queue."""
        try:
            updates_made = False
            while not self._command_queue.empty():
                command, *args = self._command_queue.get_nowait()
                if command == 'show':
                    updates_made = True
                    id_str, value_str = args
                    if id_str not in self._all_known_ids:
                        self._all_known_ids[id_str] = None
                    self._current_frame_data[id_str] = value_str
                    if self._view_mode == 'live':
                        self._update_widget(id_str)
                elif command == 'next':
                    updates_made = True
                    self._handle_next_frame()
                elif command == 'close':
                    self._on_closing()
                    return
            
            if updates_made and self._view_mode == 'live':
                # Don't use the delayed scheduler for live updates, do it directly.
                self._recalculate_all_heights()

        finally:
            if self.is_active:
                try:
                    self.root.after(100, self._process_queue)
                except tk.TclError:
                    self.is_active = False
    
    def _handle_next_frame(self):
        """Logic for when a 'next' command is received."""
        if self._current_frame_data:
            self._frames.append(self._current_frame_data)
        else:
             self._frames.append(OrderedDict())
        self._current_frame_data = OrderedDict()

        if self._view_mode == 'paused':
            self._update_frame_info_label()
            self._update_button_states()
            return
        
        self._view_frame_index = len(self._frames)
        self._redraw_all_widgets()
        self._update_frame_info_label()
        self._update_button_states()


    def _redraw_all_widgets(self):
        """Clears and redraws all data widgets based on the current view index and known IDs."""
        for id_str in self._display_widgets:
            self._display_widgets[id_str][0].destroy()
            self._display_widgets[id_str][1].destroy()
        self._display_widgets.clear()
        
        for id_str in self._all_known_ids:
            self._create_widget_pair(id_str)
            self._update_widget(id_str)
        
        # After a full redraw, always recalculate heights.
        self._recalculate_all_heights()

    def _find_last_value(self, id_str, start_frame_index):
        """Searches backwards through frames to find the last known value for an ID."""
        for i in range(start_frame_index, -1, -1):
            if id_str in self._frames[i]:
                return self._frames[i][id_str]
        return ""

    def _create_widget_pair(self, id_str):
        """Creates and grids the Label and Text widgets for a given ID."""
        if id_str in self._display_widgets:
            return
            
        label = tk.Label(self._scrollable_frame, text=f"{id_str}:", anchor='nw', 
                         bg=self._colors['label_bg'], fg=self._colors['fg'],
                         padx=5, pady=2, font=self._default_font)
        
        value_text = tk.Text(self._scrollable_frame, height=1, wrap=tk.WORD,
                             bg=self._colors['widget_bg'], fg=self._colors['fg'],
                             font=self._monospace_font, borderwidth=0,
                             highlightthickness=0)
        
        value_text.tag_configure("add", background=self._colors['highlight_add'])
        value_text.tag_configure("change", background=self._colors['highlight_change'])
        value_text.tag_configure("stale", foreground=self._colors['stale_fg'])
        
        row = len(self._display_widgets)
        label.grid(row=row, column=0, sticky='nsew', pady=(2,0))
        value_text.grid(row=row, column=1, sticky='nsew', pady=(2,0))
        self._scrollable_frame.grid_columnconfigure(1, weight=1)
        self._display_widgets[id_str] = (label, value_text)

    def _update_widget(self, id_str: str):
        """Updates a specific widget's content. Does NOT handle height."""
        if id_str not in self._display_widgets:
            self._create_widget_pair(id_str)
            
        _, value_text = self._display_widgets[id_str]
        
        new_value = ""
        old_value = ""
        is_stale = False

        if self._view_mode == 'live':
            is_fresh = id_str in self._current_frame_data
            if is_fresh:
                new_value = self._current_frame_data[id_str]
                if len(self._frames) > 0:
                    old_value = self._frames[-1].get(id_str, "")
            else:
                is_stale = True
                new_value = self._find_last_value(id_str, len(self._frames) - 1)
        
        else: # Paused mode
            frame_idx = self._view_frame_index
            if 0 <= frame_idx < len(self._frames):
                is_present_in_frame = id_str in self._frames[frame_idx]
                if is_present_in_frame:
                    new_value = self._frames[frame_idx][id_str]
                    if frame_idx > 0:
                       old_value = self._frames[frame_idx - 1].get(id_str, "")
                else:
                    is_stale = True
                    new_value = self._find_last_value(id_str, frame_idx)

        value_text.configure(state=tk.NORMAL)
        value_text.delete('1.0', tk.END)

        if is_stale:
            value_text.insert('1.0', new_value, 'stale')
        else:
            matcher = difflib.SequenceMatcher(None, old_value, new_value, autojunk=False)
            for tag, i1, i2, j1, j2 in matcher.get_opcodes():
                if tag == 'equal':
                    value_text.insert(tk.END, new_value[j1:j2])
                elif tag == 'insert':
                    value_text.insert(tk.END, new_value[j1:j2], 'add')
                elif tag == 'replace':
                    value_text.insert(tk.END, new_value[j1:j2], 'change')
        
        value_text.configure(state=tk.DISABLED)

    # --- Button Callbacks ---
    def _toggle_pause(self):
        if self._view_mode == 'live':
            self._view_mode = 'paused'
            self._pause_button.config(text="Catch Up to Live Display")
            self._view_frame_index = len(self._frames) - 1
        else:
            self._view_mode = 'live'
            self._pause_button.config(text="Pause Live Display")
            self._view_frame_index = len(self._frames)
        self._redraw_all_widgets()
        self._update_frame_info_label()
        self._update_button_states()

    def _prev_frame(self):
        if self._view_frame_index > 0:
            self._view_frame_index -= 1
            self._redraw_all_widgets()
            self._update_frame_info_label()
            self._update_button_states()

    def _next_frame(self):
        if self._view_frame_index < len(self._frames) - 1:
            self._view_frame_index += 1
            self._redraw_all_widgets()
            self._update_frame_info_label()
            self._update_button_states()

    def _update_button_states(self):
        if self._view_mode == 'paused':
            self._prev_button.config(state=tk.NORMAL if self._view_frame_index > 0 else tk.DISABLED)
            self._next_button.config(state=tk.NORMAL if self._view_frame_index < len(self._frames) - 1 else tk.DISABLED)
        else:
            self._prev_button.config(state=tk.DISABLED)
            self._next_button.config(state=tk.DISABLED)
            
    def _update_frame_info_label(self):
        total_frames = len(self._frames)
        if self._view_mode == 'live':
            self._frame_info_label.config(text=f"Display: Live (Frame {total_frames})")
        else:
            display_idx = self._view_frame_index + 1 if total_frames > 0 else 0
            self._frame_info_label.config(text=f"Display: Paused (Frame {display_idx} of {total_frames})")


# ==============================================================================
# EXAMPLE USAGE
# ==============================================================================
if __name__ == '__main__':
    
    class RobotState:
        def __init__(self, x, y, status):
            self.x = x
            self.y = y
            self.status = status
        
        def __repr__(self):
            return f"RobotState(x={self.x}, y={self.y}, status='{self.status}')"

    print("Starting example control loop...")
    print("Close the debugger window to stop the program.")

    dw = DebuggerWindow()
    dw.startup()

    loop_counter = 0
    message = "Initializing"
    robot = RobotState(0, 0, "Booting")
    long_string = "This is a very long string designed to demonstrate the new word wrapping feature of the text widgets in the debugger window. Resize the window to see it in action."
    
    try:
        while dw.is_active:
            dw.next()

            loop_counter += 1
            robot.x += 1
            if loop_counter % 10 == 0:
                robot.y += 1
                robot.status = "Turning"
                long_string += " The robot is now turning, which adds even more text to this already quite lengthy string."
            elif loop_counter % 5 == 0:
                robot.status = "Moving"

            if loop_counter % 25 == 0:
                message += "."
            
            dw.show("Loop Counter", loop_counter)
            dw.show("Message", message)
            dw.show("Robot", robot)
            dw.show("Long String Example", long_string)
            
            if loop_counter % 3 == 0:
                dw.show("Periodic Value", f"Seen on loop {loop_counter}")
            
            if loop_counter > 20:
                dw.show("New Value", "This value appeared late")
            
            print(f"Main loop iteration: {loop_counter}")
            time.sleep(0.2)
            
    except (tk.TclError, KeyboardInterrupt):
        print("\nWindow closed or program interrupted.")
    finally:
        dw.close()
        print("Example loop finished.")

