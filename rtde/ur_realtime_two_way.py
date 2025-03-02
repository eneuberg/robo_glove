"""
Copright: Yuvesh Aubeeluck
Cologne UAS
"""


from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface
from rtde_control import RTDEControlInterface
import time, json
from tqdm import tqdm

"""
Updated class with fewer methods just for receiving UR data
and for controlling the gripper.
Polyscope codes to test the methods:
- "rtde_test.urp" or "rtde_test_gripper.urp" (Installation: "latest28_10_2024") - to read the sensor data
- "rtde_test_input.urp" (Installation: "latest28_10_2024")  - to control gripper and see force

Switch UR to Remote Control, Then run the Python script, then switch UR to local
to run the UR script, then back again to Remote.
"""

# 00-30-d6-23-87-0f

class URRealTimeSimplified:
    def __init__(self, test_target=False, reset=False):
        self.ip = "192.168.12.145"
        self.reset_rtde=reset

        self.condition=False
        self.connected=False
        self.safety_status=False
        self.ur_motion=False
        self.num_outputs = 20

        # UPDATED NOV 2024 / InnoFaktur
        self.double_var_rtde=0.0
        self.gripper_force=0
        self.gripper_width=None

        #if not self.reset_rtde and self.ur_ping():
        if not self.reset_rtde:
            try:
                print("Setting up communication...")
                time.sleep(0.1)
                self.rtde_io = RTDEIOInterface(self.ip)
                print(f"rtde_io set up complete. Out: {self.rtde_io}")
                time.sleep(0.1)
                self.rtde_r = RTDEReceiveInterface(self.ip)
                print(f"rtde_r set up complete. Out: {self.rtde_r}")
                time.sleep(0.1)
                self.rtde_c = RTDEControlInterface(self.ip)
                print(f"rtde_c set up complete. Out: {self.rtde_c}")
                self.condition=True
                #print(f"Robot is operational: {self.ur_ready()}")
            except RuntimeError as e:
                self.condition=False
                # print caught runtimrerror
                print(f"RuntimeError: {e}")
                print("No control possible on Robot. Enable remote control on Robot teach pendant.")
        elif self.reset_rtde and self.ur_ping():
            self.rtde_io = RTDEIOInterface(self.ip)
            self.rtde_r = RTDEReceiveInterface(self.ip)
            self.rtde_c = RTDEControlInterface(self.ip)
            self.rtde_r.stopScript()
            self.rtde_io.stopScript()
            self.rtde_c.stopScript()
        else:
            print("Robot parameters are not initialized.")

    def obtain_int_var(self, ur_register):
        obtained_int_data=self.rtde_r.getOutputIntRegister(ur_register)
        return obtained_int_data
    def obtain_double_var(self, ur_register):
        obtained_double_data=self.rtde_r.getOutputDoubleRegister(ur_register)
        return obtained_double_data
    def set_int_var(self, ur_register, value):
        return self.rtde_io.setInputIntRegister(ur_register, value)
    def set_double_var(self, ur_register, value):
        return self.rtde_io.setInputDoubleRegister(ur_register, value)

    def read_2FG_force(self):
        return self.obtain_double_var(14)
    def read_2FG_width(self):
        return self.obtain_double_var(15)
    
    def set_2FG_gripperwidth(self, interior_width=None, status='default'):
        
        if status=='close' and interior_width==None:
            interior_width=0 #23
        elif status=='open' and interior_width==None:
            interior_width=36
        elif status=='default' and interior_width>0 and interior_width<37: # 22 AND 37
            interior_width=interior_width

        #return self.set_int_var(20,interior_width)
        return self.set_double_var(21,interior_width)

    def pseudo_force_control(self, sensitivity=1):
        while True:
            if self.read_2FG_width() < 20.0:
                self.set_2FG_gripperwidth(20.0)
            self.gripper_width = self.read_2FG_width()
            self.gripper_force = self.read_2FG_force()
            print(f"Current width: {self.gripper_width} - current force: {self.gripper_force}")
            
            if self.read_2FG_force() > 5:
                self.gripper_width-=1
                print(f"width decrease: {self.gripper_width}")
                if self.gripper_width<=20:
                    continue
                else:
                    self.set_2FG_gripperwidth(int(self.gripper_width))
            
            elif self.read_2FG_force() < -5:
                self.gripper_width+=1
                print(f"width increase: {self.gripper_width}")
                if self.gripper_width>=35:
                    continue
                else:
                    self.set_2FG_gripperwidth(int(self.gripper_width))
            
        


if __name__ == "__main__":
    hubbie = URRealTimeSimplified()
    hubbie.pseudo_force_control()
    while False:
        print(hubbie.read_2FG_force())
        #gripper_width = int(input("Enter gripper width (between 0 and 37):"))
        #hubbie.set_2FG_gripperwidth(interior_width=gripper_width)
        time.sleep(1)