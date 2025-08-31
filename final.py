import math
import time
class uav:
    R = 6371000
    max_v,min_v=80,10
    integral=0
    prev_err=0
    def __init__(self,kp,ki,kd,strt_psn,checkpoints,dt):
        self.kp=float(kp)
        self.ki=float(ki)
        self.kd=float(kd)
        self.curr=strt_psn
        self.c_v=10.0
        self.target = checkpoints[0]
        self.checkpoints = checkpoints
        self.dleft=999
        self.dt=float(dt) #interval after which we want to update the velocity
    

    def find_distance(self,c1,c2):
        #using haversin formula to calc distance b/w coords
        lat1,lon1 = c1
        lat2,lon2 = c2
        la1_d,la2_d=math.radians(lat1),math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(la1_d) * math.cos(la2_d) * math.sin(dlambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return self.R*c
    

    def pid(self):
        err = self.find_distance(self.curr,self.target)
        self.integral += err*self.dt
        derivative = (err - self.prev_err)/self.dt if self.dt > 0 else 0
        self.prev_err = err
        v_final = self.kp*err + self.ki*self.integral + self.kd*derivative
        return max(self.min_v, min(self.max_v, abs(v_final)))

    
    def move_real(self):
        self.c_v=self.pid()        
        dist_step = self.c_v * self.dt / self.R

        lat1,lon1 = self.curr[0] * math.pi / 180 , self.curr[1] * math.pi / 180
        lat2,lon2 = self.target[0] * math.pi / 180 , self.target[1] * math.pi / 180

        dlon = lon2 - lon1
        x = math.cos(lat2) * math.sin(dlon)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(x, y)
        new_lat = math.asin(math.sin(lat1) * math.cos(dist_step) + math.cos(lat1) * math.sin(dist_step) * math.cos(bearing))
        new_lon = lon1 + math.atan2(math.sin(bearing) * math.sin(dist_step) * math.cos(lat1), math.cos(dist_step) - math.sin(lat1) * math.sin(new_lat))
        

        self.curr[0] = new_lat * 180 / math.pi
        self.curr[1] = new_lon * 180 / math.pi


    def run(self):
        s_time=time.time()
        t = 0.0
        last_print_time = 0.0
        while self.checkpoints:
            self.target = self.checkpoints[0]
            self.move_real()
            dleft = self.find_distance(self.curr, self.target)
            self.dleft = dleft
            print(f"t={int(t)}s  Distance to target: {self.dleft:.2f} m  Velocity: {self.c_v:.2f} m/s")

            if dleft <= 5:
                print(f"Reached {self.target} <-----")
                self.checkpoints.pop(0)
            time.sleep(self.dt)
            t += self.dt

            if t - last_print_time >= 5:
                # print(f"t={int(t)}s  Distance to target: {self.dleft:.2f} m  Velocity: {self.c_v:.2f} m/s")
               with open('log.txt','a') as f:
                f.write(f't={int(t)}s , Distance to target({self.target}): {self.dleft}, Velocity: {self.c_v}m/s\n')
                last_print_time = t




if __name__ =='__main__':
    checkpoints = [[28.744444,77.138056],[28.723611,77.113333]]
    strt_psn=[28.748611,77.117222]
    drone = uav(kp=0.2,ki=0.00005, kd=1,dt=1,strt_psn=strt_psn,checkpoints=checkpoints)
    open("log.txt", "w").close()
    drone.run()
