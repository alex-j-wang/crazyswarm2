from NODE.NODE import *


class RigidHybrid(ODEF):
    def __init__(self, quad_params, device):  # quadparams is only a placeholder, not actually used
        super(RigidHybrid, self).__init__()
        self.lin1 = nn.Linear(9, 32)
        self.lin2 = nn.Linear(32, 6)
        self.tanh = nn.Tanh()
        self.device = device

    def forward(self, z):
        bs,_,_ = z.size()
        z = z.squeeze(1)
        x = z[:, :6]  # 6D state
        u = z[:, 6:]  # 3D input
        
        x_dot = torch.zeros([bs, 6]).to(self.device)
        x_dot[:, :3] = x[:, 3:]
        x_dot[:, 3:] = u
        
        x_dot_nn = self.tanh(self.lin1(z))
        x_dot_nn = self.lin2(x_dot_nn)

        x_dot_hybrid = x_dot + x_dot_nn
        out = torch.cat([x_dot_hybrid, torch.zeros([bs, 3]).to(self.device)], 1)
        #print("output shape is: ", out.shape)
        return out.unsqueeze(1)
        
