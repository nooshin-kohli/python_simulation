'''
Nooshin Kohli
'''

class BodyClass3d(object):
    
    def __init__(self):
        
        self.bodies = ['jump','hip','thigh','calf']
        return None


    def id(self, name):
        return self.bodies.index(name)
        
    def name(self, id):
        return self.bodies[id]
        
    def parent_id(self, id):
        p_id = id - 1
        
        # elif id == self.id('thigh'): p_id = self.id('')
        
        return p_id


class JointClass3d(object):
    
    def __init__(self):
        
        self.joints = ['jumper','hip_joint','thigh_joint','calf_joint']
        
        self.bodies = BodyClass3d()
        
        return None
        
    def q_i(self, name):
        return self.joints.index(name)
        
    def name(self, q_i):
        return self.joints[q_i]
        
    def parent_child_body(self, name):
        q_i = self.q_i(name)
        child = q_i + 1
        return self.bodies.parent_id(child), child
        
    def pairs(self):
    
        pairs = []
    
        # manually pair joints:
        pairs.append(['jumper', 'hip_joint'])
        pairs.append(['hip_joint', 'thigh_joint'])
        pairs.append(['thigh_joint', 'calf_joint'])
        
        return pairs