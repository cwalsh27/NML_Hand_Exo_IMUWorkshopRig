"""
  (c) Jonathan Shulgach - Cite and Notice license:
    All modifications to this code or use of it must include this notice and give credit for use.
    Credit requirements:
      All publications using this code must cite all contributors to this code.
      A list must be updated below indicating the contributors alongside the original or modified code appropriately.
      All code built on this code must retain this notice. All projects incorporating this code must retain this license text alongside the original or modified code.
      All projects incorporating this code must retain the existing citation and license text in each code file and modify it to include all contributors.
      Web, video, or other presentation materials must give credit for the contributors to this code, if it contributes to the subject presented.
      All modifications to this code or other associated documentation must retain this notice or a modified version which may only involve updating the contributor list.
    
    Primary Authors:
      - Jonathan Shulgach, PhD Student - Neuromechatronics Lab, Carnegie Mellon University

   Other than the above, this code may be used for any purpose and no financial or other compensation is required.
   Contributors do not relinquish their copyright(s) to this software by virtue of offering this license.
   Any modifications to the license require permission of the authors.
   
   Description:
      This project collects data from an MPU6050 and sends the data over a serial connection. 
      It also displays the current euler angles on a connected OLED screen
"""

from mpu_controller import MPUController

# Main entry point
if __name__ == "__main__":
    controller = MPUController(verbose=False)
    try:
        controller.start()
    except KeyboardInterrupt:
        controller.stop()