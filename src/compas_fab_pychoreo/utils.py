import logging

###########################################

def get_disabled_collisions(semantics):
    # TODO: move to semantics
    disabled_collisions = {}
    for dc in semantics.root.iter('disable_collisions'):
        link1, link2 = dc.attrib['link1'], dc.attrib['link2']
        if link1 not in disabled_collisions:
            disabled_collisions.update({link1: []})
        disabled_collisions[link1].append(link2)
    return disabled_collisions

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
