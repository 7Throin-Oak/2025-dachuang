import sys
print('Python %s on %s' % (sys.version, sys.platform))
sys.path.extend(['E:\\program\\Python WorkSpace\\myRL', '/home/chenchao/PycharmProjects/myRL', '/public/home/ac6f681fyd/ws/myRL'])

import torch
import torch.nn as nn
#from envs.real_env.data_utils import DynamicsDataset

#这是一个力补偿的网络，doosan_conmponent用到，用于对外力进行补偿
class CompensationNet(nn.Module):

    def __init__(self, device="cpu"):
        super(CompensationNet, self).__init__()
        self.device = device
        self.net = nn.Sequential(
            # nn.BatchNorm1d(7),
            nn.Linear(7, 64),
            nn.LeakyReLU(),
            nn.Linear(64, 128),
            nn.LeakyReLU(),
            nn.Linear(128, 6),
        )

    def forward(self, eef_pos_quat, eef_ex_force=None): #获取位置和外力，用于构建模型
        if len(eef_pos_quat.shape) == 1:  # [n_dof]
            eef_pos_quat = torch.FloatTensor(eef_pos_quat).reshape(1, -1).to(self.device)
            pred = self.net(eef_pos_quat)
            return pred[0].detach().cpu().numpy()
        elif len(eef_pos_quat.shape) == 2:  # [bs, n_dof], [bs, T, n_dof]
            eef_pos_quat = torch.FloatTensor(eef_pos_quat).to(self.device)
            eef_ex_force = torch.FloatTensor(eef_ex_force).to(self.device)
            pred = self.net(eef_pos_quat)
            loss = (pred - eef_ex_force).pow(2).mean()
            return eef_ex_force.mean(0).detach().cpu().numpy(), pred.mean(0).detach().cpu().numpy(), loss
        else:
            assert False


if __name__ == "__main__":
    '''
    import os
    import numpy as np
    from torch.utils.data import DataLoader
    ##from global_util.mpi import mpi_init, mpi_fork, get_device

    mpi_fork(num_workers=1)
    mpi_init(seed=None, gpu_ids=None)

    np.set_printoptions(precision=2, suppress=True)
    torch.set_printoptions(precision=2, sci_mode=False)

    device = get_device()
    compensation_net = CompensationNet(device=device)
    load_path = os.path.join(os.path.dirname(__file__), "compensation.pth")
    if os.path.isfile(load_path):
        compensation_net.load_state_dict(torch.load(load_path, map_location=torch.device("cpu")))

    compensation_net.to(device)
    optimizer = torch.optim.Adam(params=compensation_net.parameters(), lr=1e-3)
    ##dataset = DynamicsDataset(name="ee_force", frames=1)
    print(f"len_dataset: {len(dataset)}")
    dataloader = DataLoader(
        dataset=dataset,
        batch_size=128,
        num_workers=10,
        shuffle=True,
    )
    cnt = 0
    for epoch in range(100000):
        all_loss = 0.0
        for i, control_dict in enumerate(dataloader):
            cnt += 1
            gt, pred, loss = compensation_net(control_dict["eef_pos_quat"], control_dict["eef_ex_force"])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            all_loss += loss.item()

            print("\r", cnt, round(loss.item(), 3), end=" ")
            print(f"\r cnt={cnt}, loss={round(loss.item(), 3)}, {gt}, {pred}", end=" ")
            if cnt % 100 == 0 and cnt != 0:
                print(cnt, all_loss / 100, "====================")
                all_loss = 0.0
                torch.save(compensation_net.state_dict(), os.path.join(os.path.dirname(__file__), "compensation.pth"))
                #保存训练好的模型 对应于文件夹中的compensation.pth'''
