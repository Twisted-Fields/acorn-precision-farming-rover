import coloredlogs

_LOGGER_FORMAT_STRING = '%(asctime)s - %(name)-11s - %(levelname)-4s - %(message)s'
_LOGGER_DATE_FORMAT = '%m/%d/%Y %I:%M:%S.%f %p'
_LOGGER_LEVEL_INFO = 'INFO'
_LOGGER_LEVEL_DEBUG = 'DEBUG'


def AppendFIFO(list, value, max_values):
    list.append(value)
    while len(list) > max_values:
        list.pop(0)
    return list


def clamp(v, low, high):
    """clamp v to the range of [low, high]"""
    return max(low, min(v, high))


def config_logging(logger, debug=False):
    log_level = _LOGGER_LEVEL_DEBUG if debug else _LOGGER_LEVEL_INFO
    coloredlogs.install(fmt=_LOGGER_FORMAT_STRING,
                        datefmt=_LOGGER_DATE_FORMAT,
                        level=log_level,
                        logger=logger)
