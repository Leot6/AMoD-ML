import torch
import torch.nn.functional as F
import torch.utils.data as Data
import torchvision
import numpy as np
from torch import nn
from torch.autograd import Variable

# Hyper Parameters
BATCH_SIZE = 32
LR = 0.01                   # learning rate
EPSILON = 0.9               # greedy policy
GAMMA = 0.9                 # reward discount
TARGET_REPLACE_ITER = 100   # target update frequency
MEMORY_CAPACITY = 2000
env = gym.make('CartPole-v0')
env = env.unwrapped
N_ACTIONS = env.action_space.n
N_STATES = env.observation_space.shape[0]
ENV_A_SHAPE = 0 if isinstance(env.action_space.sample(), int) else env.action_space.sample().shape


class ScoreNet(nn.Module):
    def __init__(self, num_locations: int, veh_capacity: int):
        super(ScoreNet, self).__init__()
        # Layer 1: Schedule location embedding.
        self.embedding_location = nn.Embedding(num_embeddings=num_locations + 1, embedding_dim=100, padding_idx=0)

        # Layer 2: Masking layer for delay input.

        # Layer 3: LSTM
        self.lstm = nn.LSTM(input_size=101, hidden_size=200, bidirectional=True)

        # Layer 4: System time processing (the time scale in the real world)
        self.fc1_time = nn.Linear(in_features=1, out_features=100)

        # Layer 5: Schedule state processing.
        #  (locations, delays, system_time, number of new requests and the number of other vehicles
        #  that will arrive in the MAX_PICKUP_DELAY area of this schedule's next location)
        self.fc2_state = nn.Linear(in_features=302, out_features=300)
        # Layer 6: Schedule state processing.
        self.fc3_state = nn.Linear(in_features=300, out_features=300)
        # Layer 7: Output dense layer with one output for the schedule scoring task.
        self.out = nn.Linear(in_features=300, out_features=1)

    def forward(self, x):
        pass


def save_net_to_file(net: ScoreNet, file_path: str = "net.pkl"):
    torch.save(net, file_path)


def restore_net_from_file(file_path: str = "net.pkl"):
    return torch.load(file_path)


class Net(nn.Module):
    def __init__(self, ):
        super(Net, self).__init__()
        self.lstm = nn.LSTM(3, 2)
        self.fc1 = nn.Linear(N_STATES, 10)
        self.fc1.weight.data.normal_(0, 0.1)   # initialization
        self.out = nn.Linear(10, N_ACTIONS)
        self.out.weight.data.normal_(0, 0.1)   # initialization

    def forward(self, x):
        x = self.fc1(x)
        x = F.relu(x)
        actions_value = self.out(x)
        return actions_value


class DQN(object):
    def __init__(self):
        self.eval_net, self.target_net = Net(), Net()
        self.learn_step_counter = 0     # 用于 target 更新计时
        self.memory_counter = 0         # 记忆库记数
        self.memory = np.zeros((MEMORY_CAPACITY, N_STATES * 2 + 2))     # 初始化记忆库
        self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr=LR)    # torch 的优化器
        self.loss_func = nn.MSELoss()   # 误差公式

    def choose_action(self, x):
        x = torch.unsqueeze(torch.FloatTensor(x), 0)
        # input only one sample
        if np.random.uniform() < EPSILON:   # greedy
            actions_value = self.eval_net.forward(x)
            action = torch.max(actions_value, 1)[1].data.numpy()
            action = action[0] if ENV_A_SHAPE == 0 else action.reshape(ENV_A_SHAPE)  # return the argmax index
        else:   # random
            action = np.random.randint(0, N_ACTIONS)
            action = action if ENV_A_SHAPE == 0 else action.reshape(ENV_A_SHAPE)
        return action

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, [a, r], s_))
        # replace the old memory with new memory
        index = self.memory_counter % MEMORY_CAPACITY
        self.memory[index, :] = transition
        self.memory_counter += 1

    def learn(self):
        # target parameter update
        if self.learn_step_counter % TARGET_REPLACE_ITER == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())
        self.learn_step_counter += 1

        # sample batch transitions
        sample_index = np.random.choice(MEMORY_CAPACITY, BATCH_SIZE)
        b_memory = self.memory[sample_index, :]
        b_s = torch.FloatTensor(b_memory[:, :N_STATES])
        b_a = torch.LongTensor(b_memory[:, N_STATES:N_STATES+1].astype(int))
        b_r = torch.FloatTensor(b_memory[:, N_STATES+1:N_STATES+2])
        b_s_ = torch.FloatTensor(b_memory[:, -N_STATES:])

        # q_eval w.r.t the action in experience
        q_eval = self.eval_net(b_s).gather(1, b_a)  # shape (batch, 1)
        q_next = self.target_net(b_s_).detach()     # detach from graph, don't backpropagate
        q_target = b_r + GAMMA * q_next.max(1)[0].view(BATCH_SIZE, 1)   # shape (batch, 1)
        loss = self.loss_func(q_eval, q_target)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()


if __name__ == '__main__':
    dqn = DQN()

    print('\nCollecting experience...')
    for i_episode in range(400):
        s = env.reset()
        ep_r = 0
        while True:
            env.render()
            a = dqn.choose_action(s)

            # take action
            s_, r, done, info = env.step(a)

            # modify the reward
            x, x_dot, theta, theta_dot = s_
            r1 = (env.x_threshold - abs(x)) / env.x_threshold - 0.8
            r2 = (env.theta_threshold_radians - abs(theta)) / env.theta_threshold_radians - 0.5
            r = r1 + r2

            dqn.store_transition(s, a, r, s_)

            ep_r += r
            if dqn.memory_counter > MEMORY_CAPACITY:
                dqn.learn()
                if done:
                    print('Ep: ', i_episode, '| Ep_r: ', round(ep_r, 2))

            if done:
                break
            s = s_
