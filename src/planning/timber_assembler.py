import math
from compas_ghpython import draw_frame
from compas_fab.ghpython.components import create_id
from compas.geometry import Frame
from compas.geometry import Vector
from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots import Robot
from compas_fab.robots import JointConstraint

class TimberAssemblyPlanner(object):

    def __init__(self, robot, group = None, start_configuration = None, path_constraints = None, planner_id = None):
        self.robot = robot
        self.group = group or self.robot.main_group_name
        self.attached_collision_meshes = []
        self.planner_id = str(planner_id) if planner_id else "RRTConnect"
        self.path_constraints = path_constraints if path_constraints else None
        self.current_configuration = start_configuration if start_configuration else robot.zero_configuration()


    def goal_constraints(self, frame, tolerance_position = None, tolerance_xaxis = None, tolerance_yaxis = None, tolerance_zaxis = None):
        tolerance_position = tolerance_position or 0.001
        tolerance_xaxis = tolerance_xaxis or 1.0
        tolerance_yaxis = tolerance_yaxis or 1.0
        tolerance_zaxis = tolerance_zaxis or 1.0

        tolerances_axes = [
            math.radians(tolerance_xaxis),
            math.radians(tolerance_yaxis),
            math.radians(tolerance_zaxis),
        ]
        return self.robot.constraints_from_frame(frame, tolerance_position, tolerances_axes, self.group)
            
    def get_trajectory(self, target_frame, linear = False, attached_collision_meshes = None, path_constraints = None, planner_id = None):
        if path_constraints:
            self.path_constraints = list(path_constraints)
        if attached_collision_meshes:
            self.attached_collision_meshes.extend([AttachedCollisionMesh(mesh, 'robot11_tool0', touch_links = self.robot.get_link_names()) for mesh in attached_collision_meshes])
        planner_id = str(planner_id) if planner_id else "RRTConnect"

        if (self.robot.client and self.robot.client.is_connected):
            options = dict(
                    attached_collision_meshes = self.attached_collision_meshes,
                    path_constraints=self.path_constraints,
                    planner_id=self.planner_id
                    )
            if linear:
                print("Linear motion through {}".format(target_frame))
                this_trajectory = self.robot.plan_cartesian_motion(target_frame, start_configuration=self.current_configuration, group=self.group, options = options)
            else:
                goal_constraint = self.goal_constraints(target_frame)
                this_trajectory = self.robot.plan_motion(goal_constraint, start_configuration=self.current_configuration, group=self.group, options = options)

        return this_trajectory

        
    def get_motion_trajectory_configs(self, target_frame, linear = False, path_constraints = None, attach_collision_meshes = None, planner_id = None):
        motion_trajectory = self.get_trajectory(target_frame, linear, attached_collision_meshes = attach_collision_meshes, path_constraints = self.path_constraints, planner_id=planner_id)
        configs = []
        for c in motion_trajectory.points:
            configs.append(self.robot.merge_group_with_full_configuration(c, motion_trajectory.start_configuration, self.group))
            planes = []
            positions = []
            velocities = []
            accelerations = []
            frame = self.robot.forward_kinematics(c, self.group, options=dict(solver="model"))
            planes.append(draw_frame(frame))
            positions.append(c.positions)
            velocities.append(c.velocities)
            accelerations.append(c.accelerations)
        self.current_configuration = configs[-1]
        return configs

    def pick_and_place(self, target_frames, object_meshes, pickup_frame, path_constraints = None, group = None):
        print(self.robot.get_link_names())
        
        self.configurations = []
        self.current_configuration['robot11_joint_EA_Z'] = -4
        self.current_configuration['robot12_joint_EA_Y'] = -12
        if path_constraints:
            self.path_constraints = path_constraints
        else:
            self.path_constraints = [JointConstraint('robot11_joint_EA_Z', -3, 1, 1, 1.0)]

        if group:
            self.group = group
        else:
            self.group = self.robot.main_group_name
            
        self.pickup_frame = pickup_frame
        
        for index, target_frame in enumerate(target_frames):
            try: 
                self.pick(object_meshes[index])
                self.place(target_frame, Vector(0,0,-0.5))
            except:
                print("sucks to suck!")
        # return outputs if you have them; here I try it for you:
        return self.configurations
    



    def pick(self, mesh, pickup_frame = None):
        print("pick")
        if pickup_frame:
            self.pickup_frame = pickup_frame
        pickup_frame_offset = Frame(self.pickup_frame.point - self.pickup_frame.zaxis * 0.2, self.pickup_frame.xaxis, self.pickup_frame.yaxis)
        self.add_path_to_position(pickup_frame_offset)
        self.add_path_to_position([pickup_frame_offset, self.pickup_frame], linear=True)        
        self.grab_part(mesh)
        self.add_path_to_position([self.pickup_frame, pickup_frame_offset], linear=True)

    def place(self, frame, approach_vector = None):
        print("place")
        if approach_vector:
            approach_frame = Frame(frame.point - approach_vector, frame.xaxis, frame.yaxis)
            self.add_path_to_position(approach_frame)
            self.add_path_to_position([approach_frame, frame], linear=True)
        else:
            self.add_path_to_position(frame)    
        self.release_part()
        retract_frame = Frame(frame.point - frame.zaxis*0.5, frame.xaxis, frame.yaxis)
        self.add_path_to_position([frame, retract_frame], linear=True)


    def grab_part(self, meshes = None):
        self.attached_collision_meshes = None #[AttachedCollisionMesh(mesh, 'robot11_tool0', touch_links = self.robot.get_link_names()) for mesh in meshes]

    def add_path_to_position(self, frame, linear = False):
        self.configurations.extend(self.get_motion_trajectory_configs(frame, linear))

    def release_part(self):
        self.attached_beam_mesh = [] 
        #self.robot.remove attachedCollisionMesh(mesh)

    def attached_collision_meshes(self):
        return [AttachedCollisionMesh(mesh) for mesh in self.attached_beam_mesh]
