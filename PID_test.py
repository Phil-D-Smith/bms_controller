import time
import PID_controller

class PID_test:
    
    def test_pid(P = 0.2,  I = 0.0, D= 0.0, L=100):
        pid = PID_controller.PID_controller(P, I, D)

        pid.SetPoint=0.0
        pid.setSampleTime(0.01)

        END = L
        feedback = 0

        feedback_list = []
        time_list = []
        setpoint_list = []

        for i in range(1, END):
            pid.update(feedback)
            output = pid.output
            if pid.SetPoint > 0:
                feedback += (output - (1/i))
            if i>9:
                pid.SetPoint = 1
            time.sleep(0.02)

            feedback_list.append(feedback)
            setpoint_list.append(pid.SetPoint)
            time_list.append(i)
            print (*feedback_list)
            print (*setpoint_list)
            print (*time_list)
            
            
            return pid.output
            print (pid.output)


    if __name__ == "__main__":
            test_pid(1.2, 1, 0.001, L=50)
            
        
        #    test_pid(0.8, L=50)
        
    