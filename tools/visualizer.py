import pyvista as pv

class visualizer:
    def __init__(self):
        self.plotter = pv.Plotter(notebook=False)
        self.plotter.show_axes()

    def set_cam(self, pos=(-20, 0, 20), foc=(0, 0, 0)):
        self.plotter.camera.position = pos
        self.plotter.camera.focal_point = foc
        return self

    def add_pc(self, pc, ratio=1, **kargs):
        self.plotter.add_mesh(
            pv.PolyData(pc * ratio),
            render_points_as_spheres=True,
            point_size=2,
            **kargs
        )
        return self

    def add_line(self, p1, p2, ratio=1, **kargs):
        self.plotter.add_mesh(
            pv.Line(p1 * ratio, p2 * ratio),
            **kargs,
            line_width=1
        )
        return self

    def add_box(self, bound, trans, ratio=1):
        l, w, h = map(lambda x: x * ratio, bound)
        box = pv.Box(bounds=(-l/2, l/2, -w/2, w/2, -h/2, h/2))
        box = box.translate(list(map(lambda x: x * ratio, trans)))
        self.plotter.add_mesh(box, color='yellow')
        return self

    def show(self):
        self.plotter.show()
        return self

    def close(self):
        self.plotter.close()
        return None
