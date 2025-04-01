#!/usr/bin/env python3

import pinocchio as pin
import numpy as np

class pinocchioInverseKinematicsSolver:
    def __init__(self):
        urdf_filename = "models/urdf/gen3_2f85_fixed.urdf"
        self.model = pin.buildModelFromUrdf(urdf_filename)
        
        # add contact frame (middle of the gripper) to the model
        endT1 = pin.SE3(
            np.array([[1, 0, 0], 
                      [0, -1, 0], 
                      [0, 0, -1]]), 
            np.array([0, 0, -0.061525])) # end effector -> gripper base
        
        endT2 = pin.SE3(
            np.array([[0, -1, 0], 
                      [1, 0, 0], 
                      [0, 0, 1]]), 
            np.array([0, 0, 0.12])) # gripper base -> contact joint
        
        endT = endT1 * endT2
        
        last_joint_id = self.model.getJointId("joint_7")
        self.model.addFrame(
            pin.Frame(
                "gripper_frame", 
                last_joint_id, 
                0, 
                endT, 
                pin.FrameType.OP_FRAME
            )
        )
        
        self.end_effector_id = self.model.getFrameId("gripper_frame")
        self.data = self.model.createData()
        
        # Kinova joint limits
        self.joint_limits_lower = np.array([-np.inf, -128.9, -np.inf, -147.8, -np.inf, -120.3, -np.inf]) * np.pi / 180
        self.joint_limits_upper = np.array([ np.inf,  128.9,  np.inf,  147.8,  np.inf,  120.3,  np.inf]) * np.pi / 180
        
        # the following are a bunch of useful rotation matrices for gripper
        self.desiredRotation_x_pos = np.array([
            [0,  0, 1],   
            [-1, 0, 0],   
            [0, -1, 0]
        ]) # gripper pointing along the positive direction of x axis
        
        self.desiredRotation_y_neg = np.array([
            [-1, 0, 0],   
            [0, 0, -1],   
            [0, -1, 0]
        ]) # gripper pointing along the negative direction of y axis
        
        self.desiredRotation_z_neg = np.array([
            [0, -1,  0],   
            [-1, 0,  0],   
            [0,  0, -1]
        ]) # gripper pointing along the negative direction of z axis (pointing down)
        
    def solve(
        self,
        q0: np.ndarray, # initial guess
        desiredTranslation: np.ndarray, # desired position of the gripper
        desiredRotation: np.ndarray, # desired orientation of the gripper
        verbose: bool = False):
        if len(q0) != 7:
            raise ValueError("Invalid joint configuration shape")
        
        if desiredTranslation.shape != (3,):
            raise ValueError("Invalid translation vector shape")
        
        if desiredRotation.shape != (3, 3):
            raise ValueError("Invalid rotation matrix shape")
        
        # the following is directly from pinocchio official example
        # https://github.com/stack-of-tasks/pinocchio/blob/master/examples/inverse-kinematics.py
        oMdes = pin.SE3(desiredRotation, desiredTranslation)
        
        eps = 1e-6
        IT_MAX = 1000
        DT = 1e-1
        damp = 1e-12
        it = 0
        q = q0
        
        while True:
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            iMd = self.data.oMf[self.end_effector_id].actInv(oMdes)
            err = pin.log(iMd).vector 
            if np.linalg.norm(err) < eps:
                success = True
                break
            if it >= IT_MAX:
                print("Warning from pinocchioInverseKinematicsSolver: Maximum iterations reached without convergence!")
                success = False
                break
            J = pin.computeFrameJacobian(self.model, self.data, q, self.end_effector_id)
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)
            v = -J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
            q = pin.integrate(self.model, q, v * DT)
            if (not it % 10) and verbose:
                print("%d: error = %s" % (it, err.T))
            it += 1
            
        # check joint limits
        if success and \
           (np.any(q < self.joint_limits_lower) or \
            np.any(q > self.joint_limits_upper)):
            print("Warning from pinocchioInverseKinematicsSolver: Joint limits violated!")
            success = False
            
        return q, success
