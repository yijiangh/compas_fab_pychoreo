import logging

###########################################

def values_as_list(dict_data):
    vals = []
    for _, v in dict_data.items():
        if isinstance(v, list):
            vals.extend(v)
        else:
            vals.append(v)
    return vals

###########################################

def is_valid_option(options, key, default_value):
    return default_value if options is None or key not in options else options[key]

###########################################

def get_logger(name):
    logger = logging.getLogger(name)

    try:
        from colorlog import ColoredFormatter
        formatter = ColoredFormatter("%(log_color)s%(levelname)-8s%(reset)s %(white)s%(message)s",
                                     datefmt=None,
                                     reset=True,
                                     log_colors={'DEBUG': 'cyan', 'INFO': 'green',
                                                 'WARNING': 'yellow',
                                                 'ERROR': 'red', 'CRITICAL': 'red',
                                                 }
                                     )
    except ImportError:
        formatter = logging.Formatter('[%(levelname)s] %(message)s')

    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    return logger


LOG = get_logger(__name__)
