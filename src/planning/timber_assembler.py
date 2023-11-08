
import math
from compas_rhino.conversions import RhinoPlane
from compas_ghpython import draw_frame
from compas_ghpython import list_to_ghtree
from compas_fab.ghpython.components import create_id
from compas.geometry import Frame
from compas_fab.robots import AttachedCollisionMesh
from compas_rhino.geometry import RhinoMesh

class TimberAssemblyPlanner(object):

    def __init__(self, robot, group = None, start_configuration = None, path_constraints = None):
        self.robot = robot
        self.group = group or self.robot.main_group_name
        if path_constraints:
            self.path_constraints = path_constraints
        if not start_configuration:
            self.current_configuration = robot.zero_configuration()
        else:
            self.current_configuration = start_configuration

    def goal_constraints(self, plane, tolerance_position = None, tolerance_xaxis = None, tolerance_yaxis = None, tolerance_zaxis = None):
        tolerance_position = tolerance_position or 0.001
        tolerance_xaxis = tolerance_xaxis or 1.0
        tolerance_yaxis = tolerance_yaxis or 1.0
        tolerance_zaxis = tolerance_zaxis or 1.0

        frame = RhinoPlane.from_geometry(plane).to_compas_frame()
        tolerances_axes = [
            math.radians(tolerance_xaxis),
            math.radians(tolerance_yaxis),
            math.radians(tolerance_zaxis),
        ]
        return self.robot.constraints_from_frame(frame, tolerance_position, tolerances_axes, self.group)
            
    def get_trajectory(self, target_frame, attached_collision_meshes = None, path_constraints = None, planner_id = None):
        if path_constraints:
            self.path_constraints = list(path_constraints)
        attached_collision_meshes = list(attached_collision_meshes) if attached_collision_meshes else None
        planner_id = str(planner_id) if planner_id else "RRTConnect"

        if (self.robot.client and self.robot.client.is_connected and goal_constraints):
            options = dict(
                    attached_collision_meshes=self.attached_collision_meshes,
                    path_constraints=self.path_constraints,
                    planner_id=self.planner_id
                    )
            goal_constraints = self.goal_constraints(target_frame)
            trajectory = self.robot.plan_motion(goal_constraints, start_configuration=self.current_configuration, group=self.group, options = options)

        return trajectory

        
    def add_frame_trajectory_configs(self, target_frame, path_constraints = None, attach_collision_meshes = None, planner_id = None):
        print("mesh is " + str(attach_collision_meshes))
        trajectory = self.get_trajectory(target_frame, attached_collision_meshes=attach_collision_meshes, path_constraints=path_constraints, planner_id=planner_id)
        configs = []
        for c in trajectory.points:
            configs.append(self.robot.merge_group_with_full_configuration(c, trajectory.start_configuration, self.group))
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
        self.configurations = []
        self.current_configuration['robot11_joint_EA_Z'] = -4
        self.path_constraints = path_constraints
        if group:
            self.group = group
        else:
            self.group = self.robot.main_group_name
            
        self.pickup_frame = pickup_frame
        
        for index, target_frame in enumerate(target_frames):
            group = 'robot11_eaXYZ'
            self.pick(object_meshes[index])
            self.place(target_frame)
        # return outputs if you have them; here I try it for you:
        return self.configurations
    

    def pick(self, mesh, pickup_frame):
        if pickup_frame:
            self.pickup_frame = pickup_frame
        pickup_frame_offset = Frame(self.pickup_frame.Origin + self.pickup_frame.ZAxis * -0.5, self.pickup_frame.XAxis, self.pickup_frame.YAxis)
        self.path_to_position(pickup_frame_offset)
        self.linear_to_position(self.pickup_frame)        
        self.grab_part(mesh)
        self.path_to_position(pickup_frame_offset)

    def place(self, frame, approach_vector = None):
        if approach_vector:
            approach_frame = frame
            approach_frame.point = approach_frame.point - approach_vector
            self.path_to_position(approach_frame)
            self.linear_to_position(frame)
        else:
            self.path_to_position(frame)    
        self.release_part()
        retract_frame = frame
        retract_frame.point = retract_frame.point + (retract_frame.zaxis*0.5)
        self.retract_robot(retract_frame)


    def grab_part(self, mesh = None):
        pass
        #self.robot.append attachedCollisionMesh(mesh)

    def path_to_position(self, frame):
        self.configurations.append(self.add_frame_trajectory_configs(frame))

    def linear_to_position(self, frame, approach_vector):
        frame.point = frame.point+approach_vector
        self.configurations.append(self.add_frame_trajectory_configs(frame))

    def release_part(self):
        pass
        #self.robot.remove attachedCollisionMesh(mesh)
