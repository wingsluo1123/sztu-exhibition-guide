"""Worker warmup handshake, independent of ROS and actuator interfaces."""
def matcher_ready():
    from online_match_worker import MATCHER
    if MATCHER is None:
        raise RuntimeError('matching worker initializer has not completed')
    return True
