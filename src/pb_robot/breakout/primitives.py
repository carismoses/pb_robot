import time
import ss_pybullet.utils_noBase as utils
import planning
import constraints
import placements

DEBUG_FAILURE = False

##################################################

class BodyPose(object):
    def __init__(self, body, pose=None):
        if pose is None:
            pose = body.get_pose()
        self.body = body
        self.pose = pose
    def assign(self):
        self.body.set_pose(self.pose)
        return self.pose
    def __repr__(self):
        return 'p{}'.format(id(self) % 1000)


class BodyGrasp(object):
    def __init__(self, body, grasp_pose, approach_pose, robot, link):
        self.body = body
        self.grasp_pose = grasp_pose
        self.approach_pose = approach_pose
        self.robot = robot
        self.link = link
    #def constraint(self):
    #    grasp_constraint()
    def attachment(self):
        return utils.Attachment(self.robot, self.link, self.grasp_pose, self.body)
    def assign(self):
        return self.attachment().assign()
    def __repr__(self):
        return 'g{}'.format(id(self) % 1000)


class BodyConf(object):
    def __init__(self, body, configuration=None, joints=None):
        if joints is None:
            joints = body.get_movable_joints()
        if configuration is None:
            configuration = body.get_joint_positions(joints)
        self.body = body
        self.joints = joints
        self.configuration = configuration
    def assign(self):
        self.body.set_joint_positions(self.joints, self.configuration)
        return self.configuration
    def __repr__(self):
        return 'q{}'.format(id(self) % 1000)


class BodyPath(object):
    def __init__(self, body, path, joints=None, attachments=[]):
        if joints is None:
            joints = body.get_movable_joints()
        self.body = body
        self.path = path
        self.joints = joints
        self.attachments = attachments
    def bodies(self):
        return set([self.body] + [attachment.body for attachment in self.attachments])
    def iterator(self):
        # TODO: compute and cache these
        # TODO: compute bounding boxes as well
        for i, configuration in enumerate(self.path):
            self.body.set_joint_positions(self.joints, configuration)
            for grasp in self.attachments:
                grasp.assign()
            yield i
    def control(self, real_time=False, dt=0):
        # TODO: just waypoints
        if real_time:
            utils.enable_real_time()
        else:
            utils.disable_real_time()
        for values in self.path:
            for _ in utils.joint_controller(self.body, self.joints, values):
                utils.enable_gravity()
                if not real_time:
                    utils.step_simulation()
                time.sleep(dt)
    # def full_path(self, q0=None):
    #     # TODO: could produce sequence of savers
    def refine(self, num_steps=0):
        return self.__class__(self.body, planning.refine_path(self.body, self.joints, self.path, num_steps), self.joints, self.attachments)
    def reverse(self):
        return self.__class__(self.body, self.path[::-1], self.joints, self.attachments)
    def __repr__(self):
        return '{}({},{},{},{})'.format(self.__class__.__name__, self.body, len(self.joints), len(self.path), len(self.attachments))

##################################################

class ApplyForce(object):
    def __init__(self, body, robot, link):
        self.body = body
        self.robot = robot
        self.link = link
    def bodies(self):
        return {self.body, self.robot}
    def iterator(self, **kwargs):
        return []
    def refine(self, **kwargs):
        return self
    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.robot, self.body)

class Attach(ApplyForce):
    def control(self, **kwargs):
        # TODO: store the constraint_id?
        constraints.add_fixed_constraint(self.body, self.robot, self.link)
    def reverse(self):
        return Detach(self.body, self.robot, self.link)

class Detach(ApplyForce):
    def control(self, **kwargs):
        constraints.remove_fixed_constraint(self.body, self.robot, self.link)
    def reverse(self):
        return Attach(self.body, self.robot, self.link)

class Command(object):
    def __init__(self, body_paths):
        self.body_paths = body_paths

    # def full_path(self, q0=None):
    #     if q0 is None:
    #         q0 = Conf(self.tree)
    #     new_path = [q0]
    #     for partial_path in self.body_paths:
    #         new_path += partial_path.full_path(new_path[-1])[1:]
    #     return new_path

    def step(self):
        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():
                msg = '{},{}) step?'.format(i, j)
                utils.user_input(msg)
                #print(msg)

    def execute(self, time_step=0.05):
        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():
                #time.sleep(time_step)
                utils.wait_for_duration(time_step)

    def control(self, real_time=False, dt=0): # TODO: real_time
        for body_path in self.body_paths:
            body_path.control(real_time=real_time, dt=dt)

    def refine(self, **kwargs):
        return self.__class__([body_path.refine(**kwargs) for body_path in self.body_paths])

    def reverse(self):
        return self.__class__([body_path.reverse() for body_path in reversed(self.body_paths)])

    def __repr__(self):
        return 'c{}'.format(id(self) % 1000)

#######################################################

def get_stable_gen(fixed=[]): # TODO: continuous set of grasps
    def gen(body, surface):
        while True:
            pose = placements.sample_placement(body, surface)
            if (pose is None) or any(utils.pairwise_collision(body, b) for b in fixed):
                continue
            body_pose = BodyPose(body, pose)
            yield (body_pose,)
            # TODO: check collisions
    return gen


def get_ik_fn(robot, fixed=[], teleport=False, num_attempts=10):
    movable_joints = robot.get_movable_joints()
    sample_fn = planning.get_sample_fn(robot, movable_joints)
    def fn(body, pose, grasp):
        obstacles = [body] + fixed
        gripper_pose = utils.end_effector_from_body(pose.pose, grasp.grasp_pose)
        approach_pose = utils.approach_from_grasp(grasp.approach_pose, gripper_pose)
        for _ in range(num_attempts):
            robot.set_joint_positions(movable_joints, sample_fn()) # Random seed
            # TODO: multiple attempts?
            q_approach = utils.inverse_kinematics(robot, grasp.link, approach_pose)
            if (q_approach is None) or any(utils.pairwise_collision(robot, b) for b in obstacles):
                continue
            conf = BodyConf(robot, q_approach)
            q_grasp = utils.inverse_kinematics(robot, grasp.link, gripper_pose)
            if (q_grasp is None) or any(utils.pairwise_collision(robot, b) for b in obstacles):
                continue
            if teleport:
                path = [q_approach, q_grasp]
            else:
                conf.assign()
                #direction, _ = grasp.approach_pose
                #path = workspace_trajectory(robot, grasp.link, point_from_pose(approach_pose), -direction,
                #                                   quat_from_pose(approach_pose))
                path = planning.plan_direct_joint_motion(robot, conf.joints, q_grasp, obstacles=obstacles)
                if path is None:
                    if DEBUG_FAILURE: utils.user_input('Approach motion failed')
                    continue
            command = Command([BodyPath(robot, path),
                               Attach(body, robot, grasp.link),
                               BodyPath(robot, path[::-1], attachments=[grasp])])
            return (conf, command)
            # TODO: holding collisions
        return None
    return fn

##################################################

def assign_fluent_state(fluents):
    obstacles = []
    for fluent in fluents:
        name, args = fluent[0], fluent[1:]
        if name == 'atpose':
            o, p = args
            obstacles.append(o)
            p.assign()
        else:
            raise ValueError(name)
    return obstacles

def get_free_motion_gen(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            conf1.assign()
            obstacles = fixed + assign_fluent_state(fluents)
            path = planning.plan_joint_motion(robot, conf2.joints, conf2.configuration, obstacles=obstacles, self_collisions=self_collisions)
            if path is None:
                if DEBUG_FAILURE: utils.user_input('Free motion failed')
                return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)
    return fn


def get_holding_motion_gen(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2, body, grasp, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            conf1.assign()
            obstacles = fixed + assign_fluent_state(fluents)
            path = planning.plan_joint_motion(robot, conf2.joints, conf2.configuration,
                                              obstacles=obstacles, attachments=[grasp.attachment()], 
                                              self_collisions=self_collisions)
            if path is None:
                if DEBUG_FAILURE: utils.user_input('Holding motion failed')
                return None
        command = Command([BodyPath(robot, path, joints=conf2.joints, attachments=[grasp])])
        return (command,)
    return fn

##################################################

def get_movable_collision_test():
    def test(command, body, pose):
        pose.assign()
        for path in command.body_paths:
            moving = path.bodies()
            if body in moving:
                # TODO: cannot collide with itself
                continue
            for _ in path.iterator():
                # TODO: could shuffle this
                if any(utils.pairwise_collision(mov, body) for mov in moving):
                    if DEBUG_FAILURE: utils.user_input('Movable collision')
                    return True
        return False
    return test
