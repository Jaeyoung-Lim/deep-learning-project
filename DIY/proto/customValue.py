import BaseClasses as bc
import tensorflow as tf
import core


class customValue(bc.SpecializedFunction):
    input_names = ['state']
    output_names = ['value']

    def __init__(self, dtype, gs):
        super(customValue, self).__init__(dtype, gs)
        # variables
        state_dim = gs.input.shape[1]
        state = gs.input
        print(self.output_names)
        value = tf.identity(gs.output, name=self.output_names[0])
        clip_param = tf.Variable(0.2 * tf.ones(dtype=dtype, shape=[1, 1]), name='clip_param')

        # new placeholders
        value_target = tf.placeholder(dtype, shape=[None, 1], name='targetValue')

        # Assign ops.
        param_assign_placeholder = tf.placeholder(dtype, shape=[1, 1], name='param_assign_placeholder')
        tf.assign(clip_param, param_assign_placeholder, name='clip_param_assign')

       # solvers
        with tf.name_scope('trainUsingTargetValue'):
            core.square_loss_opt(dtype, value_target, value, tf.train.AdamOptimizer)