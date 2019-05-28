import os
import random
import tempfile
import logging
import numpy as np
import tensorflow as tf
import cPickle as pickle
from tensorflow.contrib import layers

from pathlib import Path

SEED = 1234
FLAGS = tf.app.flags.FLAGS


class Seq2Seq:
    def __init__(self):
        random.seed(SEED)
        self.STOP, self.UNK, self.STOP_ID, self.UNK_ID, self.bsz = "STOP", "UNK", 1, 2, 32
        self.embed_sz, self.rnn_sz, self.init, self.keep_prob = 30, 256, tf.contrib.layers.xavier_initializer(), 0.8
        self.use_attn, self.use_beam_search, self.beam_width, self.num_train_steps = True, True, 5, 4000

        self.train_eval()
        # self.cross_val()
        
    def train_eval(self):
        self.input_fn, self.feed_fn, self.test_feed_fn, self.human_feed_fn, self.ground_truth = self.make_data_fns()
        
        self.params = {'src_vocab_size': self.src_vsz,
                       'tar_vocab_size': self.tar_vsz,
                       'batch_size': self.bsz,
                       'embed_dim': self.embed_sz,
                       'rnn_size': self.rnn_sz}
        
        self.est = tf.estimator.Estimator(model_fn=self.seq2seq_model_fn, model_dir=FLAGS.model_dir, params=self.params)

        print_ground_inps = tf.train.LoggingTensorHook(['input_0'], every_n_iter=1000, formatter=self.print_formatter(['input_0'], self.src_vmap))
        print_ground_outs = tf.train.LoggingTensorHook(['output_0'], every_n_iter=1000, formatter=self.print_formatter(['output_0'], self.tar_vmap))
        print_train_preds = tf.train.LoggingTensorHook(['train_pred'], every_n_iter=1000, formatter=self.print_formatter(['train_pred'], self.tar_vmap))
        print_test_preds = tf.train.LoggingTensorHook(['predictions'], every_n_iter=1000, formatter=self.print_formatter(['predictions'], self.tar_vmap))

        self.est.train(input_fn=self.input_fn, hooks=[tf.train.FeedFnHook(self.feed_fn), print_ground_inps, print_ground_outs, print_train_preds, print_test_preds], steps=self.num_train_steps / 2)

        rev_tar = {v: k for k, v in self.tar_vmap.iteritems()}
        preds = list(self.est.predict(input_fn=self.input_fn, hooks=[tf.train.FeedFnHook(self.test_feed_fn)]))
        preds = [' '.join(map(lambda x: rev_tar.get(x, self.UNK), filter(lambda x: x > 1, p))) for p in preds]
        grounds = [' '.join(map(lambda x: rev_tar.get(x, self.UNK), filter(lambda x: x > 1, g))) for g in self.ground_truth]
        self.compute_acc(zip(grounds, preds))
        preds = list(self.est.predict(input_fn=self.input_fn, hooks=[tf.train.FeedFnHook(self.human_feed_fn)]))
        # preds = map(lambda x: np.transpose(x), preds)
        # for p in preds:
        #    print '*'
        #    for i in range(self.beam_width):
        #        print ' '.join(map(lambda x: rev_tar.get(x, self.UNK), filter(lambda x: x > 1, p[i])))
        preds = [' '.join(map(lambda x: rev_tar.get(x, self.UNK), filter(lambda x: x > 1, p))) for p in preds]
        print preds

    def cross_val(self):
        src_temp, tar_temp = tempfile.NamedTemporaryFile(delete=False), tempfile.NamedTemporaryFile(delete=False)
        with open(FLAGS.train_src, 'rb') as srcf, open(FLAGS.train_tar, 'rb') as tarf:
            src_sents, tar_sents = srcf.readlines(), tarf.readlines()
        assert len(src_sents) == len(tar_sents)
        z_sents = zip(src_sents, tar_sents)
        for _ in range(10):
            random.shuffle(z_sents)
        for src_sent, tar_sent in z_sents:
            src_temp.write(src_sent)
            tar_temp.write(tar_sent)

        src_temp.close()
        tar_temp.close()
        FLAGS.train_src, FLAGS.train_tar = src_temp.name, tar_temp.name
            
        correct, total = 0, 0
        print 'Starting {0}-fold cross validation'.format(FLAGS.folds)
        for f in xrange(FLAGS.folds):
            print 'Running cross validation fold {0}/{1}...'.format(f + 1, FLAGS.folds)
            tf.reset_default_graph()
            random.seed(SEED)
            tf.set_random_seed(SEED)

            self.input_fn, self.feed_fn, self.test_feed_fn, self.ground_truth = self.make_cv_data_fns(f)

            self.params = {'src_vocab_size': self.src_vsz,
                           'tar_vocab_size': self.tar_vsz,
                           'batch_size': self.bsz,
                           'embed_dim': self.embed_sz,
                           'rnn_size': self.rnn_sz}
        
            self.est = tf.estimator.Estimator(model_fn=self.seq2seq_model_fn, model_dir=None, params=self.params)

            print_ground_inps = tf.train.LoggingTensorHook(['input_0'], every_n_iter=1000, formatter=self.print_formatter(['input_0'], self.src_vmap))
            print_ground_outs = tf.train.LoggingTensorHook(['output_0'], every_n_iter=1000, formatter=self.print_formatter(['output_0'], self.tar_vmap))
            print_train_preds = tf.train.LoggingTensorHook(['train_pred'], every_n_iter=1000, formatter=self.print_formatter(['train_pred'], self.tar_vmap))
            print_test_preds = tf.train.LoggingTensorHook(['predictions'], every_n_iter=1000, formatter=self.print_formatter(['predictions'], self.tar_vmap))

            self.est.train(input_fn=self.input_fn, hooks=[tf.train.FeedFnHook(self.feed_fn), print_ground_inps, print_ground_outs, print_train_preds, print_test_preds], steps=self.num_train_steps)

            rev_tar = {v: k for k, v in self.tar_vmap.iteritems()}
            preds = list(self.est.predict(input_fn=self.input_fn, hooks=[tf.train.FeedFnHook(self.test_feed_fn)]))
            #preds = map(lambda x: np.transpose(x), preds)
            #grounds = [' '.join(map(lambda x: rev_tar.get(x, self.UNK), filter(lambda x: x > 1, g))) for g in self.ground_truth]
            #for g, p in zip(grounds, preds):
            #    print 'Ground: {0}'.format(g)
            #    for i in range(self.beam_width):
            #        print ' '.join(map(lambda x: rev_tar.get(x, self.UNK), filter(lambda x: x > 1, p[i])))
            preds = [' '.join(map(lambda x: rev_tar.get(x, self.UNK), filter(lambda x: x > 1, p))) for p in preds]
            grounds = [' '.join(map(lambda x: rev_tar.get(x, self.UNK), filter(lambda x: x > 1, g))) for g in self.ground_truth]
            c, t, _ = self.compute_acc(zip(grounds, preds))
            correct += c
            total += t

        os.remove(src_temp.name)
        os.remove(tar_temp.name)
        print '{0}-fold Cross Validation Accuracy: {1}/{2} = {3}%'.format(FLAGS.folds, correct, total, 100. * float(correct) / float(total))

    def compute_acc(self, zipped):
        correct, total = 0, 0
        for gt, pred in zipped:
            print "'{0}' \t '{1}'".format(gt, pred)
            if gt == pred:
                correct += 1
            total += 1
        acc = 100. * float(correct) / float(total)
        print 'Accuracy: {0}/{1} = {2}%'.format(correct, total, acc)
        return correct, total, acc

    def print_formatter(self, keys, vocab):
        rev_vmap = {v: k for k, v in vocab.iteritems()}

        def to_str(sequence):
            return ' '.join([rev_vmap.get(x, self.UNK) for x in sequence])

        def format(values):
            res = []
            for key in keys:
                if len(values[key].shape) == 1:
                    res.append("%s = %s" % (key, to_str(values[key])))
                else:
                    for i in range(values[key].shape[-1]):
                        res.append("{0}{1} = {2}".format(key, i + 1, to_str(values[key][:, i])))
            return '\n'.join(res)
        return format

    def build_vocab(self):
        if not os.path.isfile(FLAGS.vocabs_path) or FLAGS.rebuild:
            print 'Pre-built vocabularies not found and/or rebuild flag set to {0}'.format(FLAGS.rebuild)
            print 'Building vocabularies...'
            
            # Load source and target training data
            with open(FLAGS.train_src, 'rb') as f:
                src_words = f.read().strip().split()

            with open(FLAGS.train_tar, 'rb') as f:
                tar_words = f.read().strip().split()

            # Build + Return Vocabularies
            src_v, tar_v, stops = set(src_words), set(tar_words), ['PAD', self.STOP, self.UNK]
            src_map, tar_map = {w: i for i, w in enumerate(stops + list(src_v))}, {w: i for i, w in enumerate(stops + list(tar_v))}

            if FLAGS.reload_path:
                with open(FLAGS.reload_vpath, 'rb') as f:
                    src_map = pickle.load(f)
                    tar_map = pickle.load(f)

            with open(FLAGS.vocabs_path, 'wb') as f:
                pickle.dump((src_map, tar_map), f)

            with open('src_vocab.tsv', 'wb') as voc:
                for k, v in sorted(src_map.iteritems(), key=lambda x: x[1]):
                    voc.write(str(k) + '\n')

            with open('tar_vocab.tsv', 'wb') as voc:
                for k, v in sorted(tar_map.iteritems(), key=lambda x: x[1]):
                    voc.write(str(k) + '\n')
                    
            return src_map, tar_map
        else:
            print 'Pre-built vocabulary files found and/or rebuild flag set to {0}'.format(FLAGS.rebuild)
            with open(FLAGS.vocabs_path, 'rb') as f:
                return pickle.load(f)

    def make_data_fns(self):
        self.src_vmap, self.tar_vmap = self.build_vocab()

        self.src_vsz, self.tar_vsz = len(self.src_vmap), len(self.tar_vmap)
        
        def input_fn():
            inp = tf.placeholder(tf.int64, shape=[None, None], name='input')
            output = tf.placeholder(tf.int64, shape=[None, None], name='output')
            tf.identity(inp[0], 'input_0')
            tf.identity(output[0], 'output_0')
            return {'input': inp, 'output': output}, None

        def base_sampler():
            with open(FLAGS.train_src) as finput, open(FLAGS.train_tar) as foutput:
                for in_line, out_line in zip(finput, foutput):
                    yield {'input': list(reversed([self.src_vmap.get(w, self.UNK_ID) for w in in_line.split()])) + [self.STOP_ID],
                           'output': [self.tar_vmap.get(w, self.UNK_ID) for w in out_line.split()] + [self.STOP_ID]}

        def human_sampler():
            while True:
                try:
                    in_line = raw_input("Please enter a command: ")
                    yield {'input': list(reversed([self.src_vmap.get(w, self.UNK_ID) for w in in_line.split()])) + [self.STOP_ID],
                           'output': [self.STOP_ID]}
                except EOFError:
                    break

        def inf_sampler():
            b = base_sampler()
            while True:
                try:
                    yield b.next()
                except StopIteration:
                    b = base_sampler()

        def epoch_sampler():
            for e in xrange(FLAGS.epochs):
                print 'Starting Epoch {0}/{1}'.format(e + 1, FLAGS.epochs)
                b = base_sampler()
                while True:
                    try:
                        yield b.next()
                    except StopIteration:
                        break

        train_sampler, test_sampler, cli_sampler = inf_sampler(), base_sampler(), human_sampler()

        def feed_fn():
            inputs, outputs = [], []
            input_length, output_length = 0, 0
            for i in range(self.bsz):
                try:
                    rec = train_sampler.next()
                    inputs.append(rec['input'])
                    outputs.append(rec['output'])
                    input_length = max(input_length, len(inputs[-1]))
                    output_length = max(output_length, len(outputs[-1]))
                except StopIteration:
                    break

            if input_length == 0:
                raise StopIteration

            for i in range(len(inputs)):
                inputs[i] += [self.STOP_ID] * (input_length - len(inputs[i]))
                outputs[i] += [self.STOP_ID] * (output_length - len(outputs[i]))
            return {'input:0': inputs, 'output:0': outputs}

        ground_truth = []

        def test_feed_fn():
            inputs, outputs = [], []
            input_length, output_length = 0, 0
            for i in range(self.bsz):
                try:
                    rec = test_sampler.next()
                    inputs.append(rec['input'])
                    outputs.append(rec['output'])
                    ground_truth.append(rec['output'])
                    input_length = max(input_length, len(inputs[-1]))
                    output_length = max(output_length, len(outputs[-1]))
                except StopIteration:
                    break

            if input_length == 0:
                raise StopIteration

            for i in range(len(inputs)):
                inputs[i] += [self.STOP_ID] * (input_length - len(inputs[i]))
                outputs[i] += [self.STOP_ID] * (output_length - len(outputs[i]))
            return {'input:0': inputs, 'output:0': outputs}

        def human_feed_fn():
            inputs, outputs = [], []
            input_length, output_length = 0, 0
            for i in range(self.bsz):
                try:
                    rec = cli_sampler.next()
                    print rec
                    inputs.append(rec['input'])
                    outputs.append(rec['output'])
                    ground_truth.append(rec['output'])
                    input_length = max(input_length, len(inputs[-1]))
                    output_length = max(output_length, len(outputs[-1]))
                except StopIteration:
                    break

            if input_length == 0:
                raise StopIteration

            for i in range(len(inputs)):
                inputs[i] += [self.STOP_ID] * (input_length - len(inputs[i]))
                outputs[i] += [self.STOP_ID] * (output_length - len(outputs[i]))
            return {'input:0': inputs, 'output:0': outputs}

        print 'Finished data loading'
        return input_fn, feed_fn, test_feed_fn, human_feed_fn, ground_truth

    def make_cv_data_fns(self, fold_index):
        with open(FLAGS.train_src, 'rb') as srcf, open(FLAGS.train_tar, 'rb') as tarf:
            src_sents, tar_sents = map(lambda x: x.strip(), srcf.readlines()), map(lambda x: x.strip(), tarf.readlines())
            assert len(src_sents) == len(tar_sents)
            z_sents = zip(src_sents, tar_sents)
            fold_range = range(0, len(z_sents), len(z_sents) / FLAGS.folds)
            fold_range.append(len(z_sents))

            train_sents = z_sents[:fold_range[fold_index]] + z_sents[fold_range[fold_index + 1]:]
            val_sents = z_sents[fold_range[fold_index]:fold_range[fold_index + 1]]
            
            train_src, train_tar = zip(*train_sents)
            src_v = set(reduce(lambda a, b: a + b, map(lambda a: a.split(), train_src)))
            tar_v = set(reduce(lambda a, b: a + b, map(lambda a: a.split(), train_tar)))
            
            stops = ['**DUMMY**', self.STOP, self.UNK]
            src_map, tar_map = {w: i for i, w in enumerate(stops + list(src_v))}, {w: i for i, w in enumerate(stops + list(tar_v))}

            with open(FLAGS.vocabs_path, 'wb') as f:
                pickle.dump((src_map, tar_map), f)

            with open('src_vocab.tsv', 'wb') as voc:
                for k, v in sorted(src_map.iteritems(), key=lambda x: x[1]):
                    voc.write(str(k) + '\n')

            with open('tar_vocab.tsv', 'wb') as voc:
                for k, v in sorted(tar_map.iteritems(), key=lambda x: x[1]):
                    voc.write(str(k) + '\n')
                    
            self.src_vmap, self.tar_vmap = src_map, tar_map
        self.src_vsz, self.tar_vsz = len(self.src_vmap), len(self.tar_vmap)
        
        def input_fn():
            inp = tf.placeholder(tf.int64, shape=[None, None], name='input')
            output = tf.placeholder(tf.int64, shape=[None, None], name='output')
            tf.identity(inp[0], 'input_0')
            tf.identity(output[0], 'output_0')
            return {'input': inp, 'output': output}, None

        def base_train_sampler():
            for in_line, out_line in train_sents:
                yield {'input': list(reversed([self.src_vmap.get(w, self.UNK_ID) for w in in_line.split()])) + [self.STOP_ID],
                       'output': [self.tar_vmap.get(w, self.UNK_ID) for w in out_line.split()] + [self.STOP_ID]}

        def base_val_sampler():
            for in_line, out_line in val_sents:
                yield {'input': list(reversed([self.src_vmap.get(w, self.UNK_ID) for w in in_line.split()])) + [self.STOP_ID],
                       'output': [self.tar_vmap.get(w, self.UNK_ID) for w in out_line.split()] + [self.STOP_ID]}

        def inf_sampler(sampler):
            b = sampler()
            while True:
                try:
                    yield b.next()
                except StopIteration:
                    b = sampler()

        def epoch_sampler(sampler):
            for e in xrange(FLAGS.epochs):
                print 'Starting Epoch {0}/{1}'.format(e + 1, FLAGS.epochs)
                b = sampler()
                while True:
                    try:
                        yield b.next()
                    except StopIteration:
                        break

        train_sampler, test_sampler = inf_sampler(base_train_sampler), base_val_sampler()

        def feed_fn():
            inputs, outputs = [], []
            input_length, output_length = 0, 0
            for i in range(self.bsz):
                try:
                    rec = train_sampler.next()
                    inputs.append(rec['input'])
                    outputs.append(rec['output'])
                    input_length = max(input_length, len(inputs[-1]))
                    output_length = max(output_length, len(outputs[-1]))
                except StopIteration:
                    break

            if input_length == 0:
                raise StopIteration

            for i in range(len(inputs)):
                inputs[i] += [self.STOP_ID] * (input_length - len(inputs[i]))
                outputs[i] += [self.STOP_ID] * (output_length - len(outputs[i]))
            return {'input:0': inputs, 'output:0': outputs}

        ground_truth = []

        def test_feed_fn():
            inputs, outputs = [], []
            input_length, output_length = 0, 0
            for i in range(self.bsz):
                try:
                    rec = test_sampler.next()
                    inputs.append(rec['input'])
                    outputs.append(rec['output'])
                    ground_truth.append(rec['output'])
                    input_length = max(input_length, len(inputs[-1]))
                    output_length = max(output_length, len(outputs[-1]))
                except StopIteration:
                    break

            if input_length == 0:
                raise StopIteration

            for i in range(len(inputs)):
                inputs[i] += [self.STOP_ID] * (input_length - len(inputs[i]))
                outputs[i] += [self.STOP_ID] * (output_length - len(outputs[i]))
            return {'input:0': inputs, 'output:0': outputs}

        print 'Finished data loading'
        return input_fn, feed_fn, test_feed_fn, ground_truth
            
    def seq2seq_model_fn(self, mode, features, labels, params):
        src_vocab_size = params['src_vocab_size']
        tar_vocab_size = params['tar_vocab_size']
        embed_dim = params['embed_dim']
        rnn_size = params['rnn_size']

        inp = features['input']
        output = features['output']
        batch_size = tf.shape(inp)[0]
        start_tokens = tf.zeros([batch_size], dtype=tf.int64)
        train_output = tf.concat([tf.expand_dims(start_tokens, 1), output], 1)
        input_lengths = tf.reduce_sum(tf.to_int32(tf.not_equal(inp, self.STOP_ID)), 1)
        output_lengths = tf.reduce_sum(tf.to_int32(tf.not_equal(train_output, self.STOP_ID)), 1)
        input_embed = layers.embed_sequence(inp, vocab_size=src_vocab_size, embed_dim=embed_dim, scope='src_embed')
        output_embed = layers.embed_sequence(train_output, vocab_size=tar_vocab_size, embed_dim=embed_dim, scope='tar_embed')
        with tf.variable_scope('tar_embed', reuse=True):
            embeddings = tf.get_variable('embeddings')

        cell = tf.contrib.rnn.DropoutWrapper(tf.contrib.rnn.GRUCell(num_units=rnn_size), input_keep_prob=self.keep_prob, output_keep_prob=self.keep_prob)
        encoder_outputs, encoder_final_state = tf.nn.dynamic_rnn(cell, input_embed, dtype=tf.float32)

        train_helper = tf.contrib.seq2seq.TrainingHelper(output_embed, output_lengths)

        pred_helper = tf.contrib.seq2seq.GreedyEmbeddingHelper(embeddings, start_tokens=tf.to_int32(start_tokens), end_token=self.STOP_ID)

        def decode(helper, scope, train=True, reuse=None):
            with tf.variable_scope(scope, reuse=reuse):
                cell = tf.contrib.rnn.DropoutWrapper(tf.contrib.rnn.GRUCell(num_units=rnn_size), input_keep_prob=self.keep_prob, output_keep_prob=self.keep_prob)
                if train:
                    if self.use_attn:
                        attention_mechanism = tf.contrib.seq2seq.BahdanauAttention(num_units=rnn_size, memory=encoder_outputs, memory_sequence_length=input_lengths)
                        cell = tf.contrib.seq2seq.AttentionWrapper(cell, attention_mechanism, attention_layer_size=rnn_size / 2)
                    out_cell = tf.contrib.rnn.OutputProjectionWrapper(cell, tar_vocab_size, reuse=reuse)
                    if self.use_attn:
                        decoder_initial_state = out_cell.zero_state(dtype=tf.float32, batch_size=batch_size)
                        decoder_initial_state = decoder_initial_state.clone(cell_state=encoder_final_state)
                        decoder = tf.contrib.seq2seq.BasicDecoder(cell=out_cell, helper=helper, initial_state=decoder_initial_state)
                    else:
                        decoder = tf.contrib.seq2seq.BasicDecoder(cell=out_cell, helper=helper, initial_state=encoder_final_state)
                else:
                    tiled_encoder_final_state = tf.contrib.seq2seq.tile_batch(encoder_final_state, multiplier=self.beam_width)
                    if self.use_attn:
                        tiled_encoder_outputs = tf.contrib.seq2seq.tile_batch(encoder_outputs, multiplier=self.beam_width)
                        tiled_sequence_length = tf.contrib.seq2seq.tile_batch(input_lengths, multiplier=self.beam_width)

                        attention_mechanism = tf.contrib.seq2seq.BahdanauAttention(num_units=rnn_size, memory=tiled_encoder_outputs, memory_sequence_length=tiled_sequence_length)
                        cell = tf.contrib.seq2seq.AttentionWrapper(cell, attention_mechanism, attention_layer_size=rnn_size / 2)
                    out_cell = tf.contrib.rnn.OutputProjectionWrapper(cell, tar_vocab_size, reuse=reuse)
                    if self.use_attn:
                        decoder_initial_state = out_cell.zero_state(dtype=tf.float32, batch_size=batch_size * self.beam_width)
                        decoder_initial_state = decoder_initial_state.clone(cell_state=tiled_encoder_final_state)
                        decoder = tf.contrib.seq2seq.BeamSearchDecoder(cell=out_cell, embedding=embeddings, start_tokens=tf.to_int32(start_tokens), end_token=self.STOP_ID,
                                                                       initial_state=decoder_initial_state, beam_width=self.beam_width)
                    else:
                        decoder = tf.contrib.seq2seq.BeamSearchDecoder(cell=out_cell, embedding=embeddings, start_tokens=tf.to_int32(start_tokens), end_token=self.STOP_ID,
                                                                       initial_state=tiled_encoder_final_state, beam_width=self.beam_width)
                outputs = tf.contrib.seq2seq.dynamic_decode(decoder=decoder, impute_finished=train or not self.use_beam_search, maximum_iterations=2 * tf.reduce_max(output_lengths))
            return outputs[0]
        
        train_outputs = decode(train_helper, 'decode')
        if not self.use_beam_search:
            pred_outputs = decode(pred_helper, 'decode', train=True, reuse=True)

            tf.identity(train_outputs.sample_id[0], name='train_pred')
            weights = tf.to_float(tf.not_equal(train_output[:, :-1], self.STOP_ID))
            self.loss = tf.contrib.seq2seq.sequence_loss(train_outputs.rnn_output, output, weights=weights)
            train_op = layers.optimize_loss(self.loss, tf.train.get_global_step(), optimizer=params.get('optimizer', 'Adam'), learning_rate=params.get('learning_rate', 0.001),
                                            summaries=['loss', 'learning_rate'])

            tf.identity(pred_outputs.sample_id[0], name='predictions')
            return tf.estimator.EstimatorSpec(mode=mode, predictions=pred_outputs.sample_id, loss=self.loss, train_op=train_op)
        else:
            pred_outputs = decode(pred_helper, 'decode', train=False, reuse=True)
            tf.identity(train_outputs.sample_id[0], name='train_pred')
            weights = tf.to_float(tf.not_equal(train_output[:, :-1], self.STOP_ID))
            self.loss = tf.contrib.seq2seq.sequence_loss(train_outputs.rnn_output, output, weights=weights)
            train_op = layers.optimize_loss(self.loss, tf.train.get_global_step(), optimizer=params.get('optimizer', 'Adam'), learning_rate=params.get('learning_rate', 0.001),
                                            summaries=['loss', 'learning_rate'])

            tf.identity(pred_outputs.predicted_ids[0], name='predictions')
            return tf.estimator.EstimatorSpec(mode=mode, predictions=pred_outputs.predicted_ids[:, :, 0], loss=self.loss, train_op=train_op)


def main(_):
    Seq2Seq()


if __name__ == '__main__':
    tf.logging._logger.setLevel(logging.INFO)
    UpDir = Path(__file__).parents[1]
    tf.app.flags.DEFINE_string("train_src", os.path.join(str(UpDir), 'data', 'hard_pc_src_syn.txt'), "Path to source lang training data.")
    tf.app.flags.DEFINE_string("train_tar", os.path.join(str(Updir), 'data', 'hard_pc_tar_syn.txt'), "Path to target lang training data.")
    tf.app.flags.DEFINE_string("vocabs_path", "../data/nlp/processed/seq2seq_vocabs.pkl", "Path to serialized source and target lang word=>id maps")
    tf.app.flags.DEFINE_string("preproc_path", "../data/nlp/processed/seq2seq_preproc_{0}.pkl", "Format string path to serialized source and target data")
    tf.app.flags.DEFINE_integer("epochs", 10, "Number of training epochs.")
    tf.app.flags.DEFINE_integer("folds", 5, "Number of training epochs.")
    tf.app.flags.DEFINE_boolean("rebuild", True, "Indicator for whether preprocessing structures should be rebuilt or loaded from disk")
    tf.app.flags.DEFINE_string("reload_path", "", "Path to pickled language model parameters")
    tf.app.flags.DEFINE_string("model_dir", "model/tf_seq2seq", "Path for model checkpoints")
    tf.app.run()
