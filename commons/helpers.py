"""This File contains little helper functions that can be useful for different parts of the project"""
import os
import re
import math


def get_data_location():
    """returns absolute path of data folder

    Note:
        this only works when the curdir is beneathe group5 ( which should be always the case )

    Returns:
        str: absolute path to the data folder

    Raises:
        ValueError: if group5 folder could not be found ( reason is the the script was executed outside of /group5
    """

    pattern = '^(.*?group5)'

    # get current path --> needs to be beneath /group5

    #path = os.path.abspath(os.curdir)
    path = os.path.abspath(__file__)
    
    group_5_path = re.search(pattern, path)

    if group_5_path:
        data_path = os.path.join(group_5_path.group(0), 'data')
        return data_path
    else:
        raise ValueError('Could not find group5 dir, file was executed outside of /group5')
