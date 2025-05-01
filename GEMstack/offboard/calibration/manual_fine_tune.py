from transform3d import Transform
import cv2
import pyvista as pv
import numpy as np

class Scene3D:
    def __init__(self,pc,T:Transform,K,img,on_key_pressed):
        self.pc = pc
        self.Ti = T.inv
        self.img = img
        self.plotter : pv.Plotter = pv.Plotter(notebook=False)
        self.plotter.add_axes()
        self.plotter.camera.position = (-20,0,20)
        self.plotter.camera.focal_point = (0,0,0)

        self.plotter.add_mesh(pv.PolyData(pc))
        self.camera_actor = self.plotter.add_mesh(self.make_camera(K,img.shape))
        self.plotter.add_key_event('a',lambda:on_key_pressed('a'))
        self.plotter.add_key_event('s',lambda:on_key_pressed('s'))
        self.plotter.add_key_event('d',lambda:on_key_pressed('d'))
        self.plotter.add_key_event('w',lambda:on_key_pressed('w'))
        self.plotter.add_key_event('c',lambda:on_key_pressed('c'))
        self.plotter.add_key_event('z',lambda:on_key_pressed('z'))
        self.plotter.add_key_event('j',lambda:on_key_pressed('j'))
        self.plotter.add_key_event('k',lambda:on_key_pressed('k'))
        self.plotter.add_key_event('l',lambda:on_key_pressed('l'))
        self.plotter.add_key_event('i',lambda:on_key_pressed('i'))
        self.plotter.add_key_event('u',lambda:on_key_pressed('i'))
        self.plotter.add_key_event('o',lambda:on_key_pressed('i'))
    #consts
    def make_camera(self,K,img_shape) -> pv.PolyData:
        h,w,_ = img_shape
        Ki = np.linalg.inv(K)
        rec_c = pv.Rectangle((Ki@[0,0,1],Ki@[w,0,1],Ki@[0,h,1]))
        o_c = pv.Sphere(0.1)
        c = pv.merge((rec_c,o_c))
        return c

    #statefuls

    def show(self):
        self.plotter.show(interactive_update=True)
        self.camera_actor.user_matrix = self.Ti.matrix

    def stop(self):
        self.plotter.close()
    
    def update_T(self,T):
        self.Ti = T.inv
        self.camera_actor.user_matrix = self.Ti.matrix

    def update_Ti(self,Ti):
        self.Ti = Ti
        self.camera_actor.user_matrix = self.Ti.matrix
    
    def loop(self):
        self.plotter.update()

class Scene2D:
    def __init__(self,pc,T:Transform,K,img,on_key_pressed):
        self.pc = pc
        self.img = img
        self.img_shape = img.shape
        self.T = T
        self.K = K
        self.scatter = None
        cv2.startWindowThread()
        self.window = cv2.namedWindow("projection")
        def handle_event(event):
            on_key_pressed(event.key)
        self.overlay = np.zeros_like(img)
    #statefuls
    def _update(self):
        self.overlay*=0
        u,v = pc_projection(self.pc,self.T,self.K,self.img_shape)
        for uu,vv in zip(u.astype(int),v.astype(int)):
            cv2.circle(self.overlay, (uu, vv), radius=1, color=(0, 0, 255), thickness=-1)
        display = cv2.add(self.img, self.overlay)
        cv2.imshow("projection", display)

    def show(self):
        self._update()

    def stop(self):
        cv2.destroyAllWindows()

    def update_T(self,T):
        self.T = T
        self._update()

    def loop(self):
        pass

#%%
def manual_fine_tune(pc,img,T,K):
    on = True
    tu = 0.1
    ru = 0.1
    T = Transform(T)
    def on_key_pressed(key):
        nonlocal on
        nonlocal T
        print(key)
        if key == 'c':
            T @= Transform(p=(0,0,tu))
        if key == 'z':
            T @= Transform(p=(0,0,-tu))
        if key == 'a':
            T @= Transform(p=(-tu,0,0))
        if key == 'd':
            T @= Transform(p=(tu,0,0))
        if key == 'w':
            T @= Transform(p=(0,-tu,0))
        if key == 's':
            T @= Transform(p=(0,tu,0))

        if key == 'j':
            T = Transform(rpy=(0,ru,0)) @ T
        if key == 'l':
            T = Transform(rpy=(0,-ru,0)) @ T
        if key == 'i':
            T = Transform(rpy=(ru,0,0)) @ T
        if key == 'k':
            T = Transform(rpy=(-ru,0,0)) @ T
        if key == 'u':
            T = Transform(rpy=(0,0,-ru)) @ T
        if key == 'o':
            T = Transform(rpy=(0,0,ru)) @ T

        if key == 'p':
            on = False
        s3d.update_T(T)
        s2d.update_T(T)
        
    s2d = Scene2D(pc,T,K,img,on_key_pressed)
    s3d = Scene3D(pc,T,K,img,on_key_pressed)

    s2d.show()
    s3d.show()
    while on :
        s3d.loop()
        s2d.loop()
    s3d.stop()
    s2d.stop()