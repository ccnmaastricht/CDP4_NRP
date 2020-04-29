'''
Construct saccade generator for horizontal and vertical eye movement

:license: CC BY-NC-SA 4.0, see LICENSE.md
'''

from saccade_generator_single_side import saccade_generator_single_side

def construct_saccade_generator( ):
    '''
    Construct model network of saccade generator for control of two
    extraocular muscles

    Returns
    -------
    saccade_generator : dict
        dictionary containing two single side saccade generators

    '''

    '''
    saccade generator for horizonal eye movement
    '''
    # saccade generator left saccades
    LLBN_l, EBN_l, IBN_l, OPN_h = saccade_generator_single_side()
    # saccade generator right saccades
    LLBN_r, EBN_r, IBN_r, OPN_h = saccade_generator_single_side(OPN_h)

    '''
    saccade generator for vertical eye movement
    '''
    # saccade generator upward saccades
    LLBN_u, EBN_u, IBN_u, OPN_v = saccade_generator_single_side()
    # saccade generator downward saccades
    LLBN_d, EBN_d, IBN_d, OPN_v = saccade_generator_single_side(OPN_v)

    horizontal_saccade_generator = {'LLBN_l' : LLBN_l,
                                    'EBN_l' : EBN_l,
                                    'IBN_l' : IBN_l,
                                    'LLBN_r' : LLBN_r,
                                    'EBN_r' : EBN_r,
                                    'IBN_r' : IBN_r,
                                    'OPN_h' : OPN_h}

    vertical_saccade_generator = {'LLBN_u' : LLBN_u,
                                  'EBN_u' : EBN_u,
                                  'IBN_u' : IBN_u,
                                  'LLBN_d' : LLBN_d,
                                  'EBN_d' : EBN_d,
                                  'IBN_d' : IBN_d,
                                  'OPN_v' : OPN_v}

    saccade_generator = {'horizontal' : horizontal_saccade_generator,
                         'vertical' : vertical_saccade_generator}

    return saccade_generator
