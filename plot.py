#!/usr/bin/env

import matplotlib.pyplot as plt
import os
import sys
import numpy as np
from tqdm import tqdm
from seaborn import kdeplot

def collision_cone(xBot, yBot, vxBot, vyBot, axBot, ayBot, xObs, yObs, vxObs, vyObs, noise_params, dt=0.1):
    bpn = gmm_noise_samples(noise_params, 'agent', 'position', 100)
    bvn = gmm_noise_samples(noise_params, 'agent', 'velocity', 100)
    ban = gmm_noise_samples(noise_params, 'agent', 'control', 100)
    opn = gmm_noise_samples(noise_params, 'obstacle', 'position', 100)
    ovn = gmm_noise_samples(noise_params, 'obstacle', 'velocity', 100)
    xObs = xObs + vxObs*dt
    yObs = yObs + vyObs*dt
    cones = []
    for i in range(100):
        xBott = xBot + vxBot*dt + 0.5*(axBot+ban[i][0])*dt*dt
        yBott = yBot + vyBot*dt + 0.5*(ayBot+ban[i][1])*dt*dt
        vxBott = vxBot + (axBot+ban[i][0])*dt
        vyBott = vyBot + (ayBot+ban[i][1])*dt
        for j in range(100):
            x0 = xBott + bpn[i][0]
            y0 = yBott + bpn[i][1]
            xob = xObs + opn[j][0]
            yob = yObs + opn[j][1]
            x0dot = vxBott + bvn[i][0]
            y0dot = vyBott + bvn[i][1]
            xobdot = vxObs + ovn[j][0]
            yobdot = vyObs + ovn[j][1]
            R = 2
            cones.append(-(-((x0 - xob) * (x0dot - xobdot) + (y0 - yob) * (y0dot - yobdot)) ** 2 + (
                                -R ** 2 + (x0 - xob) ** 2 + (y0 - yob) ** 2) * (
                                (x0dot - xobdot) ** 2 + (y0dot - yobdot) ** 2))
                )
    return cones

def lin_collision_cone(xBot, yBot, vxBot, vyBot, axBot, ayBot, xObs, yObs, vxObs, vyObs, noise_params, delt=0.1):
    bpn = gmm_noise_samples(noise_params, 'agent', 'position', 100)
    bvn = gmm_noise_samples(noise_params, 'agent', 'velocity', 100)
    ban = gmm_noise_samples(noise_params, 'agent', 'control', 100)
    opn = gmm_noise_samples(noise_params, 'obstacle', 'position', 100)
    ovn = gmm_noise_samples(noise_params, 'obstacle', 'velocity', 100)
    cones = []
    for i in range(100):
        for j in range(100):
            mean_xr = xBot + bpn[i][0]
            mean_yr = yBot + bpn[i][1]
            mean_xob = xObs + opn[j][0]
            mean_yob = yObs + opn[j][1]
            mean_xrdot = vxBot + bvn[i][0]
            mean_yrdot = vyBot + bvn[i][1]
            mean_xrddot = axBot + ban[i][0]
            mean_yrddot = ayBot + ban[i][1]
            mean_xobdot = vxObs + ovn[j][0]
            mean_yobdot = vyObs + ovn[j][1]
            R = 2
            cones.append(-(-((-mean_xobdot + delt * mean_xrddot + mean_xrdot) * (-mean_xob + mean_xr + 0.5 * delt ** 2 * mean_xrddot + delt * mean_xrdot) + (-mean_yobdot + delt * mean_yrddot + mean_yrdot) * (-mean_yob + mean_yr + 0.5 * delt ** 2 * mean_yrddot + delt * mean_yrdot)) ** 2 + ((-mean_xobdot + delt * mean_xrddot + mean_xrdot) ** 2 + (-mean_yobdot + delt * mean_yrddot + mean_yrdot) ** 2) * ((-mean_xob + mean_xr + 0.5 * delt ** 2 * mean_xrddot + delt * mean_xrdot) ** 2 + (-mean_yob + mean_yr + 0.5 * delt ** 2 * mean_yrddot + delt * mean_yrdot) ** 2 - R ** 2)))
    return cones



def gmm_noise_samples(params, agent_type, state_type, noise_samples):
    weights = params[agent_type][state_type]['weights']
    deviations = params[agent_type][state_type]['deviations']
    means = params[agent_type][state_type]['means']
    result = []
    for i in range(len(weights)):
        result.append((np.random.randn(round(noise_samples*weights[i]), 2) * deviations[i]) + means[i])
    return np.concatenate(result)

path = {}
with open(sys.argv[1], 'r') as f:
    for line in f:
        line = line.split(',')
        try:
            path[line[0]]['x'].append(float(line[1]))
            path[line[0]]['y'].append(float(line[2]))
            path[line[0]]['vx'].append(float(line[3]))
            path[line[0]]['vy'].append(float(line[4]))
            if line[0][:3] == 'rob':
                path[line[0]]['ax'].append(float(line[5]))
                path[line[0]]['ay'].append(float(line[6]))
        except Exception:
            path[line[0]] = { 'x': [], 'y': [], 'vx': [], 'vy': [], 'ax': [], 'ay': [] }
            path[line[0]]['x'].append(float(line[1]))
            path[line[0]]['y'].append(float(line[2]))
            path[line[0]]['vx'].append(float(line[3]))
            path[line[0]]['vy'].append(float(line[4]))
            if line[0][:3] == 'rob':
                path[line[0]]['ax'].append(float(line[5]))
                path[line[0]]['ay'].append(float(line[6]))


steps = len(path[list(path.keys())[0]]['x'])

print(np.sqrt(sum(np.array(path['robot']['ax'])**2) + sum(np.array(path['robot']['ay'])**2)))

noise_params = {
    'agent': {
        'position': {
            'weights': [0.5, 0.5],
            # 'means': [-0.12, 0.12],
            'means': [-0.2, 0.2],
            # 'deviations': [0.1, 0.2]
            'deviations': [0.4, 0.64]
        },
        'velocity': {
            'weights': [0.3, 0.7],
            'means': [-0.07, 0.03],
            'deviations': [0.1, 0.6]
        },
        'control': {
            'weights': [0.4, 0.6],
            'means': [-0.9, 1],
            'deviations': [0.1, 0.6]
        }
    },
    'obstacle': {
        'position': {
            'weights': [0.6, 0.4],
            'means': [-0.5, 0.35],
            # 'deviations': [0.1, 0.1]
            'deviations': [0.25, 0.35]
        },
        'velocity': {
            'weights': [0.5, 0.5],
            'means': [-0.05, 0.05],
            'deviations': [0.15, 0.15]
        }
    },
}

linear_noise_params = {
    'agent': {
        'position': {
            'weights': [1],
            # 'means': [-0.12, 0.12],
            'means': [-0],
            # 'deviations': [0.1, 0.2]
            'deviations': [0.324]
        },
        'velocity': {
            'weights': [1],
            'means': [-0],
            'deviations': [0.1682]
        },
        'control': {
            'weights': [1],
            'means': [-0],
            'deviations': [0.08512]
        }
    },
    'obstacle': {
        'position': {
            'weights': [1],
            'means': [-0],
            'deviations': [0.2312]
        },
        'velocity': {
            'weights': [1],
            'means': [-0.],
            'deviations': [0.07]
        }
    },
}

os.system('rm -rf outputs/*')

for i in tqdm(range(80,steps), desc='Generating image frames'):
    plt.clf()
    ax = plt.gcf().gca()
    ax.set_xlim((-4, 8))
    ax.set_ylim((-8, 4))
    for key in path.keys():
        if key[:3] == 'obs':
            color = '#ffa804'
            vehicle = 'obstacle'
        else:
            color = '#059efb'
            vehicle = 'agent'
            ax.add_artist(plt.Circle((path[key]['x'][i], path[key]['y'][i]), 6, color='gray', alpha=0.1))
        for sample in gmm_noise_samples(linear_noise_params, vehicle, 'position', 50):
            center = np.array([path[key]['x'][i], path[key]['y'][i]]) + sample
            ax.add_artist(plt.Circle(center, 1, color=color, zorder=2, alpha=0.1))
        ax.add_artist(plt.Circle((path[key]['x'][i], path[key]['y'][i]), 1, zorder=100, facecolor=color, edgecolor='black'))
        plt.plot(path[key]['x'][:i], path[key]['y'][:i], color, zorder=2)
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Linearized, t = {}s'.format(i/10))
    plt.savefig('outputs/{}.png'.format(str(i).zfill(4)), dpi=300)

os.system('cd outputs && ffmpeg -r 10 -pattern_type glob -i "*.png" -c:v libx264 -vf fps=30 -pix_fmt yuv420p -s 1920x1440 output.mp4')


def desired_velocity(position, goal, velocity):
    tmp = goal - position
    tmp /= np.sqrt(tmp.__pow__(2).sum())
    return np.sqrt((tmp*1.5 - velocity).__pow__(2).sum())

des_cost = 0

for i in tqdm(range(steps), desc='Generating distribution plots'):
    plt.clf()
    cones = collision_cone(
        path['robot']['x'][i],
        path['robot']['y'][i],
        path['robot']['vx'][i],
        path['robot']['vy'][i],
        path['robot']['ax'][i],
        path['robot']['ay'][i],
        path['obstacle']['x'][i],
        path['obstacle']['y'][i],
        path['obstacle']['vx'][i],
        path['obstacle']['vy'][i],
        noise_params
    )
    lin_cones = lin_collision_cone(
        path['robot']['x'][i],
        path['robot']['y'][i],
        path['robot']['vx'][i],
        path['robot']['vy'][i],
        path['robot']['ax'][i],
        path['robot']['ay'][i],
        path['obstacle']['x'][i],
        path['obstacle']['y'][i],
        path['obstacle']['vx'][i],
        path['obstacle']['vy'][i],
        linear_noise_params

    )
    des_cost += desired_velocity(np.array([path['robot']['x'][i], path['robot']['y'][i]]), np.array([-12, 12]), np.array([path['robot']['vx'][i], path['robot']['vy'][i]]))
    ax = kdeplot(cones, label='Actual', shade=False)
    ax = kdeplot(lin_cones, label='Linearized', shade=False)
    ax.axvline(x=0)
    plt.xlim(-300, 200)
    plt.ylabel('Density values')
    plt.xlabel('$f^j_t$')
    plt.title('Distribution of $P_{f^j_t}(u_t - 1)$ at t = ' + str(i/10) + 's')
    plt.savefig('outputs/dist-{}.png'.format(str(i).zfill(4)), dpi=300)

print(des_cost)

os.system('cd outputs && ffmpeg -r 10 -pattern_type glob -i "dist-*.png" -c:v libx264 -vf fps=30 -pix_fmt yuv420p -s 1920x1440 dist-output.mp4')
