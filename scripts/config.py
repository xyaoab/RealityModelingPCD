import os
import configargparse


class ConfigParser(configargparse.ArgParser):
    def __init__(self):
        super().__init__(default_config_files=[
            os.path.join(os.path.dirname(__file__), 'default_config.yml')
        ],
                         conflict_handler='resolve')

        # yapf:disable
        # Default arguments
        self.add(
            '--name', type=str,
            help='Name of the config for the offline reconsturction system.')
        self.add(
            '--device', type=str,
            help='Device to run the system.')

        input_parser = self.add_argument_group('input')
        input_parser.add(
            '--path_dataset', type=str,
            help='Path to the dataset folder. It should contain a folder with pcd and a folder with sensor pose.')

        integration_parser = self.add_argument_group('integration')
        integration_parser.add(
            '--voxel_size', type=float,
            help='Voxel size in meter for volumetric integration.')
        integration_parser.add(
            '--block_resolution', type=int,
            help='Voxel block resolution to construction resolution^3 local blocks')
        integration_parser.add(
            '--sdf_trunc_multiplier', type=float,
            help='Truncation distance for signed distance.')
        integration_parser.add(
            '--block_count', type=int,
            help='Pre-allocated voxel block count for volumetric integration.')
        integration_parser.add(
            '--step_size', type=int,
            help='Ray marching step size along the ray.')
        integration_parser.add(
            '--tangential_step_size', type=int,
            help='Ray marching tangential plane step size.')

    def get_config(self):
        config = self.parse_args()
        return config


if __name__ == '__main__':
    # Priority: command line > custom config file > default config file
    parser = ConfigParser()
    parser.add(
        '--config',
        is_config_file=True,
        help='YAML config file path. Please refer to default_config.yml as a '
        'reference. It overrides the default config file, but will be '
        'overridden by other command line inputs.')
    config = parser.get_config()
    print(config)
