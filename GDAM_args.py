import argparse


def get_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument("--node_vicinity", help="Radius of the vicinity in which not to add new nodes",
                        type=float, default=2.5)
    parser.add_argument("--deleted_node_vicinity",
                        help="Radius of the vicinity of deleted nodes in which not to add new nodes",
                        type=float, default=0.75)
    parser.add_argument("--launchfile", default='multi_robot_scenario.launch',
                        help="Name of the launch file")
    parser.add_argument("--min_in", help="Minimum laser measurement at which to detect a collision",
                        type=float, default=0.62)
    parser.add_argument("--side_min_in",
                        help="Minimum laser measurement at which to detect a collision at the sides",
                        type=float, default=0.62)
    parser.add_argument("--delete_nodes_range", help="Distance from an obstacle at which to delete a node",
                        type=float, default=0.3)
    parser.add_argument("--acceleration_low", help="Maximum acceleration at low speed",
                        type=float, default=0.009)
    parser.add_argument("--acceleration_high", help="Maximum acceleration at high speed",
                        type=float, default=0.012)
    parser.add_argument("--deceleration_low", help="Maximum Deceleration at low speed",
                        type=float, default=0.016)
    parser.add_argument("--deceleration_high", help="Maximum Deceleration at low speed",
                        type=float, default=0.022)
    parser.add_argument("--angular_acceleration", help="Maximum angular Acceleration/Deceleration",
                        type=float, default=0.016)
    parser.add_argument("--nr_of_nodes", help="Number of nodes to keep in memory",
                        type=int, default=400)
    parser.add_argument("--nr_of_closed_nodes", help="Number of closed nodes to keep in memory",
                        type=int, default=500)
    parser.add_argument("--nr_of_deleted_nodes", help="Number of deleted nodes to keep in memory",
                        type=int, default=500)
    parser.add_argument("--update_rate", help="Rate at which to update the current goal node",
                        type=int, default=50)
    parser.add_argument("--remove_rate", help="Rate at which to remove an unreachable goal node",
                        type=int, default=700)
    parser.add_argument("--stddev_threshold", help="Standard deviation threshold to recognize a frozen robot",
                        type=float, default=0.16)
    parser.add_argument("--freeze_rate", help="Number of steps over which to calculate the standard deviation",
                        type=int, default=400)


    parser.add_argument("--x", help="X coordinate of the goal",
                        type=float, default=50.0)
    parser.add_argument("--y", help="Y coordinate of the goal",
                        type=float, default=0.0)

    return parser.parse_args()


d_args = get_arguments()
