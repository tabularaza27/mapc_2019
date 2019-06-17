"""This File contains little helper functions that can be useful for different parts of the project"""
import os
import re
import math


def get_commons_location():
    """returns absolute path of utils folder

    Note:
        this only works when the curdir is beneathe group5 ( which should be always the case )

    Returns:
        str: absolute path to the utils folder

    Raises:
        ValueError: if group5 folder could not be found ( reason is the the script was executed outside of /group5
    """

    pattern = '^(.*?group5)'

    # get current path --> needs to be beneath /group5

    #print("AAAAAAAAAAAAAAAAAAAAAAAAAAA: " + os.path.abspath(os.curdir).__str__())
    #path = os.path.abspath(os.curdir)
    # path = "/home/ale/Desktop/EIT/AppAI/mapc_workspace/src/group5"
    path = "/home/kaijeggle/dev/Uni/AAIP/mapc_workspace/src/group5"
    
    group_5_path = re.search(pattern, path)

    if group_5_path:
        utils_path = os.path.join(group_5_path.group(0), 'commons')
        return utils_path
    else:
        raise ValueError('Could not find group5 dir, file was executed outside of /group5')
