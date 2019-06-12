#! /usr/bin/env python

import rospy
import vrep
from geometry_msgs.msg import Vector3
import numpy as np


count=0
dt=0.001
track_hand = []
track_target = []
joint_names = ['Revolute_joint','shoulder', 'elbow']
joint_handles=[]
target_handle=1
_=0
joint_target_velocities=np.zeros(len(joint_names))

def callback(msg):

        global count
        global dt
        global _


	if clientID != -1 and count== 0: # if we connected successfully
       		print ('Connected to remote API server')

        # --------------------- Setup the simulation

        	vrep.simxSynchronous(clientID,True)

                global joint_names

        # joint target velocities discussed below
                global joint_target_velocities
       		joint_target_velocities = np.ones(len(joint_names)) * 10000.0

                global joint_handles

        # get the handles for each joint and set up streaming
        	joint_handles = [vrep.simxGetObjectHandle(clientID,
            	   name, vrep.simx_opmode_blocking)[1] for name in joint_names]

        # get handle for target and set up streaming
                global target_handle
        	_, target_handle = vrep.simxGetObjectHandle(clientID,
                        'target', vrep.simx_opmode_blocking)

        
        	vrep.simxSetFloatingParameter(clientID,
                	vrep.sim_floatparam_simulation_time_step,
                	dt, # specify a simulation time step
                	vrep.simx_opmode_oneshot)

        # --------------------- Start the simulation

        # start our simulation in lockstep with our code
        	vrep.simxStartSimulation(clientID,
                	vrep.simx_opmode_blocking)
                print("Starting Simulation")



	
        if clientID!=-1 and count >= 0 and count<6: # run for 1 simulated second

                target_xyz=[msg.x, msg.y, msg.z]
		 # store for plotting
                global track_target

                track_target.append(np.copy(target_xyz))


            # get the (x,y,z) position of the target
        	vrep.simxSetObjectPosition(clientID, target_handle, -1, target_xyz, vrep.simx_opmode_blocking)
                if _ !=0 : raise Exception("Deu ruim parceiro")

		target_xyz = np.asarray(target_xyz)
            	
            
            	q = np.zeros(len(joint_handles))
            	dq = np.zeros(len(joint_handles))
            	for ii,joint_handle in enumerate(joint_handles):
                # get the joint angles
                	_, q[ii] = vrep.simxGetJointPosition(clientID,
                        joint_handle,
                        vrep.simx_opmode_blocking)
                	if _ !=0 : raise Exception()
                # get the joint velocity
                	_, dq[ii] = vrep.simxGetObjectFloatParameter(clientID,
                        	joint_handle,
                        	2012, # parameter ID for angular velocity of the joint
                        	vrep.simx_opmode_blocking)
                	if _ !=0 : raise Exception()

            	L = np.array([0, .42, .225]) # arm segment lengths

            	xyz = np.array([L[0]*np.cos(q[0])+L[1]*np.cos(q[0])*np.cos(q[1])+L[2]*np.cos(q[0])*np.cos(q[1])*np.cos(q[2])- \
			L[2]*np.cos(q[0])*np.sin(q[1])*np.sin(q[2]),
                	L[0]*np.cos(q[0])+L[1]*np.cos(q[1])*np.sin(q[0])+L[2]*np.sin(q[0])*np.cos(q[1])*np.cos(q[2])- \
			L[2]*np.sin(q[0])*np.sin(q[1])*np.sin(q[2]),
                            # have to add .1 offset to z position
                        L[1]*np.sin(q[1])+L[2]*(np.cos(q[1])*np.sin(q[2])+np.cos(q[2])*np.sin(q[1]))+.14])

                global track_hand

                track_hand.append(np.copy(xyz)) #store for plotting

            # calculate the Jacobian for the hand
                JEE = np.zeros((3,3))
            	JEE[0,2] = -L[2] * np.sin(q[1]+q[2])*np.cos(q[0]);
    	    	JEE[1,2] = -L[2] * np.sin(q[1]+q[2])*np.sin(q[0]);
    	    	JEE[2,2] = L[2]*np.cos(q[1]+q[2]);
		JEE[0,1] = -np.cos(q[0])*(L[2]*np.sin(q[1]+q[2])+L[1]*np.sin(q[1]));
    		JEE[1,1] = -np.sin(q[0])*(L[2]*np.sin(q[1]+q[2])+L[1]*np.sin(q[1]));
    		JEE[2,1] = L[2] * np.cos(q[1]+q[2]) + L[1]*np.cos(q[1]);
    		JEE[0,0] = -np.sin(q[0])*(L[0]+JEE[2][1]);
    		JEE[1,0] =np.cos(q[0])*(L[0]+JEE[2][1]);

            # get the Jacobians for the centres-of-mass for the arm segments
                JCOM1 = np.zeros((6,3))

                JCOM1[0,0] = L[0] * -np.sin(q[0])/2;
                JCOM1[1,0] = L[0] * np.cos(q[0])/2;
                JCOM1[5,0] = 1.0;

                JCOM2 = np.zeros((6,3))

                JCOM2[0,1] = L[1] * (np.cos(q[1]) * np.sin(q[0]) - np.cos(q[0]) * np.sin(q[1]))/2;
                JCOM2[1,1] = L[1] * (-np.cos(q[1]) * np.cos(q[0]) - np.sin(q[1]) * np.sin(q[0]))/2;
                JCOM2[4,1] = 1.0;
                JCOM2[0,0] = L[1]*(np.cos(q[0])*np.sin(q[1])-np.cos(q[1])*np.sin(q[0]))/2-L[0]*np.sin(q[0]);
                JCOM2[1,0] = L[1] * (np.sin(q[0])*np.sin(q[1]) + np.cos(q[0])*np.cos(q[1]))/2 + 2*JCOM1[1][0];
                JCOM2[4,0] = 1.0;

                JCOM3 = np.zeros((6,3))

                JCOM3[0,2] = L[2]*(np.cos(q[2])*np.sin(q[0]) - np.cos(q[0])*np.cos(q[1])*np.sin(q[2]))/2;
                JCOM3[1,2] = L[2]*(-np.cos(q[2])*np.cos(q[0]) - np.cos(q[1])*np.sin(q[0])*np.sin(q[2]))/2;
                JCOM3[2,2] = -L[2]*np.sin(q[1])*np.sin(q[2])/2;
                JCOM3[4,2] = 1.0;
                JCOM3[0,1] = L[2] * -np.cos(q[0])*np.cos(q[2])*np.sin(q[1])/2 - L[1]*np.cos(q[0])*np.sin(q[1]);
                JCOM3[1,1] = L[2] * -np.cos(q[2])*np.sin(q[0])*np.sin(q[1])/2 - L[1]*np.sin(q[0])*np.sin(q[1]);
                JCOM3[2,1] = (L[2]*np.cos(q[1])*np.cos(q[2])/2)+L[1]*np.cos(q[1]);
                JCOM3[4,1] = 1.0;
                JCOM3[0,0] = L[2]*(np.cos(q[0])*np.sin(q[2]) - np.cos(q[1])*np.cos(q[2])*np.sin(q[0]))/2 -L[1]*np.cos(q[1])*np.sin(q[0])- \
                                L[0]*np.sin(q[0]);
                JCOM3[1,0] = L[2]*(np.sin(q[0])*np.sin(q[2]) + np.cos(q[0])*np.cos(q[1])*np.cos(q[2]))/2 + \
			 L[1]*np.cos(q[0])*np.cos(q[1])+2*JCOM1[1][0];
                JCOM3[4,0] = 1.0;

		m1 = 0.15 #from VREP
		M1 = np.diag([m1, m1, m1, .001, .001, .001])
            	m2 = 1.171 # from VREP
           	M2 = np.diag([m1, m1, m1, .008, .008, .001])
            	m3 = .329 # from VREP
            	M3 = np.diag([m2, m2, m2, 5e-4, 5e-4, 1e-4])

            # generate the mass matrix in joint space
            	Mq = np.dot(JCOM1.T, np.dot(M1, JCOM1)) + \
                 	np.dot(JCOM2.T, np.dot(M2, JCOM2)) + \
		 	np.dot(JCOM3.T, np.dot(M3, JCOM3))

            # compensate for gravity
            	gravity = np.array([0, 0, -9.81, 0, 0, 0,])
            	Mq_g = np.dot(JCOM1.T, np.dot(M1, gravity)) + \
                    	np.dot(JCOM2.T, np.dot(M2, gravity)) + \
		    	np.dot(JCOM2.T, np.dot(M2, gravity))

            	Mx_inv = np.dot(JEE, np.dot(np.linalg.inv(Mq), JEE.T))
            	Mu,Ms,Mv = np.linalg.svd(Mx_inv)
            # cut off any np.singular values that could cause control problems
            	for i in range(len(Ms)):
                	Ms[i] = 0 if Ms[i] < 1e-5 else 1./float(Ms[i])
            # numpy returns U,S,V.T, so have to transpose both here
            	Mx = np.dot(Mv.T, np.dot(np.diag(Ms), Mu.T))

            # calculate desired movement in operational (hand) space
            	kp = 100
            	kv = np.sqrt(kp)
            	u_xyz = np.dot(Mx, kp * (target_xyz - xyz))

            	u = np.dot(JEE.T, u_xyz) - np.dot(Mq, kv * dq) - Mq_g
            	u *= -1 # because the joints on the arm are backwards

           	for ii,joint_handle in enumerate(joint_handles):
                # the way we're going to do force control is by setting
                # the target velocities of each joint super high and then
                # controlling the max torque allowed (yeah, i know)

                # get the current joint torque
                	_, torque = \
                    	vrep.simxGetJointForce(clientID,
                            	joint_handle,
                            	vrep.simx_opmode_blocking)
                	if _ !=0 : raise Exception()

                # if force has changed signs,
                # we need to change the target velocity sign
                	if np.sign(torque) * np.sign(u[ii]) < 0:
                    		joint_target_velocities[ii] = \
                            		joint_target_velocities[ii] * -1
                    		vrep.simxSetJointTargetVelocity(clientID,
                            		joint_handle,
                            		joint_target_velocities[ii], # target velocity
                            		vrep.simx_opmode_blocking)
                	if _ !=0 : raise Exception()

                # and now modulate the force
                	vrep.simxSetJointForce(clientID,
                        	joint_handle,
                        	abs(u[ii]), # force to apply
                        	vrep.simx_opmode_blocking)
                	if _ !=0 : raise Exception()

            # move simulation ahead one time step
            	vrep.simxSynchronousTrigger(clientID)
                count += dt
                print("Cheguei aqui uma vez")
        elif clientID != -1 and count>=6:
		vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

        # Before closing the connection to V-REP,
        #make sure that the last command sent out had time to arrive.
                vrep.simxGetPingTime(clientID)

        # Now close the connection to V-REP:
        	vrep.simxFinish(clientID)

	else:
		raise Exception('Failed connecting to remote API server')
	

        return


def main():

	global clientID
	
	rospy.init_node("dym_vrep",anonymous= True)
	vrep.simxFinish(-1)

	if count==0:
		clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)
		
	sub = rospy.Subscriber("left_hand_joint", Vector3, callback)
	rospy.spin()
	return clientID


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
            print("Deu Ruim")
            pass

        finally:

            global clientID
            global track_target
            global track_hand
            # stop the simulation
            vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

            # Before closing the connection to V-REP,
            # make sure that the last command sent out had time to arrive.
            vrep.simxGetPingTime(clientID)

            # Now close the connection to V-REP:
            vrep.simxFinish(clientID)
            print('connection closed...')

            import matplotlib as mpl
            from mpl_toolkits.mplot3d import Axes3D
            import matplotlib.pyplot as plt

            track_hand = np.array(track_hand)
            track_target = np.array(track_target)

            np.savetxt('dymI.txt',track_target,'%.4f')
            np.savetxt('dymO.txt',track_hand,'%.4f')


            fig = plt.figure()
            ax = fig.gca(projection='3d')
                    # plot start point of hand
            ax.plot([track_hand[0,0]], [track_hand[0,1]], [track_hand[0,2]], 'bx', mew=10)
                     # plot trajectory of hand
            ax.plot(track_hand[:,0], track_hand[:,1], track_hand[:,2])
        # plot trajectory of target
            ax.plot(track_target[:,0], track_target[:,1], track_target[:,2], 'rx', mew=10)

            ax.set_xlim([-1, 1])
            ax.set_ylim([-.5, .5])
            ax.set_zlim([0, 1])
            ax.legend()

            plt.show()




    	


