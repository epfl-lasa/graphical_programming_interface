import numpy as np

ft_upright = {
    'force': [0.88956034, 0.020819, -8.9924709],
    'torque': [0.00550106, -0.01819782, 0.00019738],
}

ft_sideways_1 = {
    'force': [-7.48023766, 0.241739, -0.6155169],
    'torque': [0.00217806, -0.59203582, 0.00144438],
}

ft_sideways_2 = {
    'force': [0.96484934, 8.438038, -0.5392609],
    'torque': [-0.56747194, -0.00197782, 0.00308238],
}


ft_list = [ft_upright, ft_sideways_1, ft_sideways_2]


def force_evaluation():
    for ii in range(len(ft_list)):
        print(ft_list[ii]['force'])
        print('Force {}: {}'.format(ii, np.round(np.linalg.norm(ft_list[ii]['force']), 3)))

        print(ft_list[ii]['torque'])
        print('Torque {}: {}'.format(ii, np.round(np.linalg.norm(ft_list[ii]['torque']), 3)))
        
        print('')


def torque_evalution():
    for ft in [ft_sideways_1, ft_sideways_2]:
        force = np.linalg.norm(ft['force'])

        # print('force {} N'.format(round(force, 3)))
        
        torque_xy = ft['torque'][:2]
        
        # print('torque xy N', torque_xy, 3)
        torque_xy_norm  = np.linalg.norm(torque_xy)
        # print('torque xy {} N'.format(round(torque_xy_norm, 3)))

        # T = F x dist
        # dist = T/f
        dist_centermass_in_meter = torque_xy_norm / force

        print('dist of centerMass: {} m'.format(round(dist_centermass_in_meter, 4)))

        # Distance to center of mass is approximately: [0, 0, 0.073 m]
        # frame_id = 'ee_iiwa'

        #
        # Torque: induced_torce = dist_centermass x Force [x <=> cross-product]
        #
        
        
if (__name__) == "__main__":
    force_evaluation()

    torque_evalution()
    pass
