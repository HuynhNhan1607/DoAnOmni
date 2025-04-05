import tkinter as tk
from server_gui import ServerGUI

if __name__ == "__main__":
    root = tk.Tk()
    gui = ServerGUI(root)
    root.mainloop()