import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import fcl

class World:

    def __init__(self, x_min, x_max, v_min, v_max, Pset):
        self.x_min = x_min
        self.x_max = x_max
        self.v_min = v_min
        self.v_max = v_max
        self.Pset = Pset
        self.fcl_manager = fcl.DynamicAABBTreeCollisionManager()
        objs = []
        for p in self.Pset:
            objs.append(self.rec2fcl(p))
        self.fcl_manager.registerObjects(objs)
        self.fcl_manager.setup()

    def rec2fcl(self, rec):
        box = fcl.Box(rec[2], rec[3], 1.0)
        tf = fcl.Transform([rec[0], rec[1], 0])
        return fcl.CollisionObject(box, tf)

    def point2fcl(self, p):
        point = fcl.Cylinder(0.01, 1)
        tf = fcl.Transform([p[0], p[1], 0])
        return fcl.CollisionObject(point, tf)

    def isValidPoint(self, s_q):
        # check if the sampled point is inside the world"
        if not (self.x_min[0] < s_q[0] < self.x_max[0]
                and self.x_min[1] < s_q[1] < self.x_max[1]
                and self.v_min[0] < s_q[2] < self.v_max[0]
                and self.v_min[1] < s_q[3] < self.v_max[1]):
            return False

        # 一对many
        fclPoint = self.point2fcl(s_q)

        req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
        rdata = fcl.CollisionData(request=req)
        self.fcl_manager.collide(fclPoint, rdata, fcl.defaultCollisionCallback)
        # print('Collision between manager 1 and Mesh?: {}'.format(rdata.result.is_collision))
        # print('Contacts:')
        # for c in rdata.result.contacts:
        #     print('\tO1: {}, O2: {}'.format(c.o1, c.o2))

        return not rdata.result.is_collision

    def isValid(self, q_set):
        # check validity for multiple points.
        # will be used for piecewize path consited of multiple points

        # for q in q_set:
        #     if not self.isValidPoint(q):
        #         return False
        # return True
        if len(q_set.shape) < 2:
            return self.isValidPoint(q_set)
        manager_q = fcl.DynamicAABBTreeCollisionManager()
        qs = []
        for q in q_set:
            qs.append(self.point2fcl(q))
        manager_q.registerObjects(qs)
        req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
        rdata = fcl.CollisionData(request=req)
        self.fcl_manager.collide(manager_q, rdata, fcl.defaultCollisionCallback)
        # print('Collision between manager 1 and Mesh?: {}'.format(rdata.result.is_collision))
        # print('Contacts:')
        # for c in rdata.result.contacts:
        #     print('\tO1: {}, O2: {}'.format(c.o1, c.o2))
        return not rdata.result.is_collision

    def drawRec(self, p, ax):
        rect = mpl.patches.Rectangle((p[0]-p[2]/2, p[1]-p[3]/2), p[2], p[3])
        ax.add_patch(rect)

    def show(self, ax):
        p1 = [self.x_min[0], self.x_min[1]]
        p2 = [self.x_min[0], self.x_max[1]]
        p3 = [self.x_max[0], self.x_max[1]]
        p4 = [self.x_max[0], self.x_min[1]]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], "k-")
        ax.plot([p2[0], p3[0]], [p2[1], p3[1]], "k-")
        ax.plot([p3[0], p4[0]], [p3[1], p4[1]], "k-")
        ax.plot([p4[0], p1[0]], [p4[1], p1[1]], "k-")
        for P in self.Pset:
            self.drawRec(P, ax)
