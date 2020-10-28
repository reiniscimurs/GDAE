from GDAM_env import ImplementEnv
import tensorflow as tf
import tflearn
import numpy as np
from GDAM_args import d_args


class ActorNetwork(object):

    def __init__(self, sess):
        self.sess = sess
        self.inputs, self.out, self.scaled_out, self.im_out = self.create_actor_network()

        self.network_params = tf.trainable_variables()

    def create_actor_network(self):
        inputs = tflearn.input_data(shape=[None, 23])
        net = tflearn.fully_connected(inputs, 800)
        net = tflearn.activations.relu(net)
        net = tflearn.fully_connected(net, 600)
        net = tflearn.activations.relu(net)
        im_out = inputs

        out = tflearn.fully_connected(net, 2, activation='tanh')

        scaled_out = tf.multiply(out, [1, 1])
        return inputs, out, scaled_out, im_out

    def predict(self, inputs):
        return self.sess.run(self.scaled_out, feed_dict={
            self.inputs: inputs
        })


def test(env, actor):

    while True:
        action = [0.0, 0.0]
        s2, toGoal = env.step(action)
        s = np.append(s2, toGoal)
        s = np.append(s, action)

        while True:

            a = actor.predict([s])
            aIn = a
            aIn[0,0] = (aIn[0,0]+1)/4
            s2, toGoal = env.step(aIn[0])
            s = np.append(s2, a[0])
            s = np.append(s, toGoal)


def main(env):

    sess = tf.InteractiveSession()

    with tf.name_scope("ActorNetwork"):
        actor = ActorNetwork(sess)

    init_op = tf.global_variables_initializer()
    sess.run(init_op)
    a_net_params = []
    for variable in tf.trainable_variables():
        a_net_params.append(variable)

    for idx, v in enumerate(a_net_params):
        print("  var {:3}: {:15}   {}".format(idx, str(v.get_shape()), v.name))
    saver = tf.train.Saver(a_net_params, max_to_keep=1)
    checkpoint = tf.train.get_checkpoint_state("/home/reinis/gym-gazebo/gym_gazebo/envs/mod/demo")
    saver.restore(sess, checkpoint.model_checkpoint_path)

    test(env, actor)


env = ImplementEnv(d_args)
main(env)

