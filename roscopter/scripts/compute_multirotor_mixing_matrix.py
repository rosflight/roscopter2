import numpy as np
import argparse
import os

import yaml
import io

def compute_mixing_matrix(file:dict, output_mixer_type:str) -> dict[str:float]:
    # Extract the data from the file
    num_motors = file['num_motors']
    motor_positions = file['motor_positions']
    motor_orientations = file['motor_orientations']
    motor_directions = file['motor_directions']

    mixing_matrix = np.zeros((6,num_motors))

    # Compute the mixing matrix parameters for each motor
    for i in range(num_motors):
        T = file['prop_ct'] * file['air_density'] * file['prop_diameter']**4 / (4*np.pi**2) * \
            np.array(motor_orientations[i])
        
        Q = np.cross(np.array(motor_positions[i]).reshape((1,-1)), T.reshape((1,-1))) + \
            file['prop_cq'] * file['air_density'] * file['prop_diameter']**5 / (4*np.pi**2) * motor_directions[i] * np.array(motor_orientations[i])

        # Save the values in the mixing matrix
        mixing_matrix[:3,i] = T
        mixing_matrix[3:,i] = Q
    
    print(f'\nMixing matrix:\n{mixing_matrix}')

    # Invert the mixing matrix
    mixing_matrix_inverted = np.linalg.pinv(mixing_matrix)

    print('\n-----------------------------\n')
    print(f'Inverted mixing matrix:\n{np.round(mixing_matrix_inverted,2)}')
    print('\n-----------------------------\n')

    # Save the inverted mixing matrix to the specified file location
    output = {}
    for i in range(mixing_matrix_inverted.shape[0]):
        for j in range(mixing_matrix_inverted.shape[1]):
            output[output_mixer_type + '_MIXER_' + str(j) + '_' + str(i)] = float(np.round(mixing_matrix_inverted[i,j], 4))

    return output, num_motors

def format_matrix(output:dict, num_motors:int) -> dict:
    output_formatted = {'root': []}
    for key, val in output.items():
        param_entry = {'name': key, 'type': 9, 'value': val}
        output_formatted['root'].append(param_entry)

    return output_formatted

def format_header(output_formatted:dict, num_motors:int) -> dict:
    # Format the header values for the mixer
    # Used in the firmware to determine output type and default PWM frequency
    for i in range(num_motors):     # The number of mixer outputs in rosflight firmware
        param_entry = {'name': 'PRI_MIXER_OUT_' + str(i), 'type': 6, 'value': 2}
        output_formatted['root'].append(param_entry)

        param_entry = {'name': 'PRI_MIXER_PWM_' + str(i), 'type': 9, 'value': 490.0}
        output_formatted['root'].append(param_entry)

    for i in range(num_motors, 10):     # The number of mixer outputs in rosflight firmware
        param_entry = {'name': 'PRI_MIXER_OUT_' + str(i), 'type': 6, 'value': 0}    # There is no header for the secondary mixer
        output_formatted['root'].append(param_entry)

        param_entry = {'name': 'PRI_MIXER_PWM_' + str(i), 'type': 9, 'value': 490.0}
        output_formatted['root'].append(param_entry)

    return output_formatted


def main(param_file:str, output_file:str, output_mixer_type:str, append_output:bool) -> int:
    if param_file == None:
        print('Error! Please specify name of parameter file. Run with \'-h\' for more information.')
        return -1
    
    with open(param_file) as stream:
        try:
            file = yaml.safe_load(stream)
        except yaml.YAMLError as e:
            print('Failure while loading YAML file!')
            print(e)
            return -1

    print('Computing mixing matrix and inverse...')
    output, num_motors = compute_mixing_matrix(file, output_mixer_type)

    # Format the output from the mixer for the ROSflight_io node
    output_formatted = format_matrix(output, num_motors)
    
    # Format the header if computing the secondary matrix
    if output_mixer_type == 'PRI':
        output_formatted = format_header(output_formatted, num_motors)

    # Write the output file
    if append_output:
        file_action = 'a'
    else:
        file_action = 'w'
        
    with open(output_file, file_action) as ostream:
        try:
            yaml.safe_dump(output_formatted['root'], ostream, default_flow_style=False)
        except yaml.YAMLError as e:
            print('Failure while writing YAML file!')
            print(e)
            return -1


    print(f'Write to file complete. File location: {output_file}')
    return 0

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Script to compute and save mixing matrix parameters')
    parser.add_argument('-f', '--param-file', type=str, default=None, help='Location of the frame configuration file. Required')
    parser.add_argument('-o', '--output-file', type=str, default=os.path.join(os.getcwd(), 'mixing_params.yaml'), help='Location to save output mixing parameters. Defaults to current working directory')
    parser.add_argument('-t', '--output-mixer-type', type=str, default='PRI', choices=['PRI', 'SEC'], help='Whether to compute the primary or secondary matrix parameters. Defaults to primary.')
    parser.add_argument('-a', '--append-output', action='store_true', help='Whether to append the computed string to the file instead of overwriting. Defaults to true.')

    args = vars(parser.parse_args())

    print('\nScript arguments:')
    for key, val in args.items():
        print(key + ':', val)
    print('\n')

    main(**args)
