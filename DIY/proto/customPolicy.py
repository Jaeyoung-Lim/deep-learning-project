import functions.Policy as pc
import tensorflow as tf
import core
import Utils
from operator import mul
from functools import reduce
import numpy as np


class customPolicy(pc.Policy):
    def __init__(self, dtype, gs):
        # shortcuts
        action_dim = int(gs.output.shape[-1])
        state_dim = int(gs.input.shape[-1])
        action = tf.identity(gs.output, name=self.output_names[0])
        # standard deviation layer
        with tf.name_scope('stdevconcatOutput'):
            wo = tf.Variable(tf.zeros(shape=[1, action_dim], dtype=dtype), name='W',
                             trainable=True)  # Log standard deviation
            action_stdev = tf.identity(tf.exp(wo), name='stdev')

        gs.l_param_list.append(wo)

        super(customPolicy, self).__init__(dtype, gs)

        stdev_assign_placeholder = tf.placeholder(dtype, shape=[1, action_dim], name='Stdev_placeholder')
        Stdev_assign = tf.assign(wo, tf.log(stdev_assign_placeholder), name='assignStdev')
        Stdev_get = tf.exp(wo, name='getStdev')

        tangent_in = tf.placeholder(dtype,  name='tangent')
        old_stdv = tf.placeholder(dtype, shape=[1, action_dim], name='stdv_o')
        old_action_in = tf.placeholder(dtype, name='sampledAction')
        old_action_noise_in = tf.placeholder(dtype, name='actionNoise')
        advantage_in = tf.placeholder(dtype, name='advantage')

        tangent_ = tf.reshape(tangent_in, [1, -1])
        old_action_sampled = tf.reshape(old_action_in, [-1, action_dim])
        old_action_noise = tf.reshape(old_action_noise_in, [-1, action_dim])
        advantage = tf.reshape(advantage_in, shape=[-1], name='test')

        # Algorithm params
        util = Utils.Utils(dtype)

        with tf.name_scope('Algo'):
            logp_n = util.log_likelihood(action, action_stdev, old_action_sampled)
            logp_old = util.log_likelihood(old_action_noise, old_stdv)
            ratio = tf.exp(logp_n - logp_old)
            ent = tf.reduce_sum(wo + .5 * tf.cast(tf.log(2.0 * np.pi * np.e), dtype=dtype), axis=-1)
            mean_ent = tf.reduce_mean(ent)

            with tf.name_scope('TRPO'):
                # Surrogate Loss
                surr = tf.reduce_mean(tf.multiply(ratio, advantage))
                loss = tf.identity(surr + mean_ent, name='loss')
                policy_gradient = tf.identity(util.flatgrad(surr, gs.l_param_list), name='Pg')  # flatgrad

                # Hessian Vector Product
                meanfixed = tf.stop_gradient(action)
                stdfixed = tf.stop_gradient(action_stdev)
                kl_ = tf.reduce_mean(util.kl_divergence(meanfixed, stdfixed, action, action_stdev))
                dkl_dth = tf.identity(util.flatgrad(kl_, gs.l_param_list))

                def getfvp(tangent):
                    temp = tf.reduce_sum(tf.multiply(dkl_dth, tangent))
                    return util.flatgrad(temp, gs.l_param_list)

                # Conjugate Gradient Descent
                out1, out2 = util.CG_tf(getfvp, tangent_, 100, 1e-15)
                Ng = tf.identity(out1, name='Cg')
                err = tf.identity(out2, name='Cgerror')