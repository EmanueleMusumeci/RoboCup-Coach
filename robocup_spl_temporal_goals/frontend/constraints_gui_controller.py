import tkinter as tk
import tkinter.font as tkFont

class ConstraintGUIController:
    def __init__(self, root):
        factor = 2
        font_factor = 1
        #setting title
        root.title("Constraint Input")
        #setting window size
        width=factor*524
        height=factor*249
        screenwidth = root.winfo_screenwidth()
        screenheight = root.winfo_screenheight()
        alignstr = '%dx%d+%d+%d' % (width, height, (screenwidth - width) / 2, (screenheight - height) / 2)
        root.geometry(alignstr)
        root.resizable(width=False, height=False)

        self.ListBox_AvailableRobots=tk.Listbox(root)
        self.ListBox_AvailableRobots["borderwidth"] = "1px"
        ft = tkFont.Font(family='Times',size=font_factor*10)
        self.ListBox_AvailableRobots["font"] = ft
        self.ListBox_AvailableRobots["fg"] = "#333333"
        self.ListBox_AvailableRobots["justify"] = "center"
        self.ListBox_AvailableRobots.place(x=factor*20,y=factor*40,width=factor*100,height=factor*193)
        self.ListBox_AvailableRobots["selectmode"] = "single"
        self.ListBox_AvailableRobots["setgrid"] = "False"

        self.Label_AvailableRoles=tk.Label(root)
        self.Label_AvailableRoles["cursor"] = "arrow"
        ft = tkFont.Font(family='Times',size=font_factor*10)
        self.Label_AvailableRoles["font"] = ft
        self.Label_AvailableRoles["fg"] = "#333333"
        self.Label_AvailableRoles["justify"] = "left"
        self.Label_AvailableRoles["text"] = "Available roles;"
        self.Label_AvailableRoles.place(x=factor*20,y=factor*10,width=factor*121,height=factor*35)

        self.TextBox_NewConstraint=tk.Entry(root)
        self.TextBox_NewConstraint["borderwidth"] = "1px"
        ft = tkFont.Font(family='Times',size=font_factor*10)
        self.TextBox_NewConstraint["font"] = ft
        self.TextBox_NewConstraint["fg"] = "#333333"
        self.TextBox_NewConstraint["justify"] = "center"
        self.TextBox_NewConstraint["text"] = ""
        self.TextBox_NewConstraint.place(x=factor*150,y=factor*180,width=factor*253,height=factor*33)

        self.Button_AddConstraint=tk.Button(root)
        self.Button_AddConstraint["bg"] = "#efefef"
        ft = tkFont.Font(family='Times',size=font_factor*10)
        self.Button_AddConstraint["font"] = ft
        self.Button_AddConstraint["fg"] = "#000000"
        self.Button_AddConstraint["justify"] = "center"
        self.Button_AddConstraint["text"] = "Add constraint"
        self.Button_AddConstraint.place(x=factor*420,y=factor*180,width=factor*84,height=factor*30)
        self.Button_AddConstraint["command"] = self.Button_AddConstraint_command

        self.ListBox_CurrentConstraints=tk.Listbox(root)
        self.ListBox_CurrentConstraints["borderwidth"] = "1px"
        ft = tkFont.Font(family='Times',size=font_factor*10)
        self.ListBox_CurrentConstraints["font"] = ft
        self.ListBox_CurrentConstraints["fg"] = "#333333"
        self.ListBox_CurrentConstraints["justify"] = "center"
        self.ListBox_CurrentConstraints.place(x=factor*150,y=factor*40,width=factor*253,height=factor*77)

        self.Label_CurrentConstraints=tk.Label(root)
        ft = tkFont.Font(family='Times',size=font_factor*10)
        self.Label_CurrentConstraints["font"] = ft
        self.Label_CurrentConstraints["fg"] = "#333333"
        self.Label_CurrentConstraints["justify"] = "center"
        self.Label_CurrentConstraints["text"] = "Current constraints:"
        self.Label_CurrentConstraints.place(x=factor*130,y=factor*10,width=factor*152,height=factor*30)

        self.Label_InsertConstraint=tk.Label(root)
        ft = tkFont.Font(family='Times',size=font_factor*10)
        self.Label_InsertConstraint["font"] = ft
        self.Label_InsertConstraint["fg"] = "#333333"
        self.Label_InsertConstraint["justify"] = "left"
        self.Label_InsertConstraint["text"] = "Insert constraint:"
        self.Label_InsertConstraint.place(x=factor*150,y=factor*150,width=factor*103,height=factor*30)

        self.Button_ShowBehavior=tk.Button(root)
        self.Button_ShowBehavior["bg"] = "#efefef"
        ft = tkFont.Font(family='Times',size=font_factor*10)
        self.Button_ShowBehavior["font"] = ft
        self.Button_ShowBehavior["fg"] = "#000000"
        self.Button_ShowBehavior["justify"] = "center"
        self.Button_ShowBehavior["text"] = "Show behavior"
        self.Button_ShowBehavior.place(x=factor*420,y=factor*60,width=factor*85,height=factor*30)
        self.Button_ShowBehavior["command"] = self.Button_ShowBehavior_command

    def Button_AddConstraint_command(self):
        print("command")


    def Button_ShowBehavior_command(self):
        print("command")

if __name__ == "__main__":
    root = tk.Tk()
    app = ConstraintGUIController(root)
    root.mainloop()
