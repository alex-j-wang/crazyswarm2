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
        

class RigidHybridAdditiveForgetting(ODEF):
    def __init__(self):  # quadparams is only a placeholder, not actually used
        super(RigidHybridAdditiveForgetting, self).__init__()
        self.layers = [nn.Linear(9, 20),
                       nn.Tanh(),
                       nn.Linear(20, 6)]
        
        for i in range(len(self.layers)):
            if str(self.layers[i])[:6] == 'Linear':
                torch.nn.init.xavier_uniform_(self.layers[i].weight)
                torch.nn.init.zeros_(self.layers[i].bias)
    
        self.nn_model = nn.ModuleList(self.layers)
        self.n_old_layers = 0
        self.nn_pool_size = 3
    
    def cascade(self):
        self.n_old_layers = len(self.nn_model)
        if self.n_old_layers == self.nn_pool_size * 3:
            self.n_old_layers = self.n_old_layers - 3
        new_layers = [nn.Linear(9, 16),
                      nn.Tanh(),
                      nn.Linear(16, 6)]
        
        for i in range(len(new_layers)):
            if str(self.layers[i])[:6] == 'Linear':
                torch.nn.init.xavier_uniform_(new_layers[i].weight)
                torch.nn.init.zeros_(new_layers[i].bias)

        # adding new layers to the nn module
        self.nn_model.extend(new_layers)
        # only keep fixed number of layers
        self.nn_model = self.nn_model[-self.nn_pool_size*3:]
 

    def forward(self, z):
        bs,_,_ = z.size()
        z = z.squeeze(1)
        x = z[:, :6]  # 6D state
        u = z[:, 6:]  # 3D input
        
        x_dot = torch.zeros([bs, 6])
        x_dot[:, :3] = x[:, 3:]
        x_dot[:, 3:] = u
         
        temp = z
        for i, l in enumerate(self.nn_model):
            temp = l(temp)
            if (i + 1)%3 == 0:
                # exponential weighting. Old layer small weight
                exp_weight = torch.exp(torch.tensor((i+1-len(self.nn_model))/3))
                x_dot += exp_weight * temp
                if (i+1) != len(self.nn_model):
                    temp = z
        
        out = torch.cat([x_dot, torch.zeros([bs, 3])], 1)
        #print("output shape is: ", out.shape)
        return out.unsqueeze(1)


class RigidHybridAdditive(ODEF):
    def __init__(self):  # quadparams is only a placeholder, not actually used
        super(RigidHybridAdditive, self).__init__()
        self.layers = [nn.Linear(9, 20),
                       nn.Tanh(),
                       nn.Linear(20, 6)]
        
        for i in range(len(self.layers)):
            if str(self.layers[i])[:6] == 'Linear':
                torch.nn.init.xavier_uniform_(self.layers[i].weight)
                torch.nn.init.zeros_(self.layers[i].bias)
    
        self.nn_model = nn.ModuleList(self.layers)
        self.n_old_layers = 0
    
    def cascade(self):
        self.n_old_layers = len(self.nn_model)
        new_layers = [nn.Linear(9, 16),
                      nn.Tanh(),
                      nn.Linear(16, 6)]
        
        for i in range(len(new_layers)):
            if str(self.layers[i])[:6] == 'Linear':
                torch.nn.init.xavier_uniform_(new_layers[i].weight)
                torch.nn.init.zeros_(new_layers[i].bias)

        self.nn_model.extend(new_layers)
 

    def forward(self, z):
        bs,_,_ = z.size()
        z = z.squeeze(1)
        x = z[:, :6]  # 6D state
        u = z[:, 6:]  # 3D input
        
        x_dot = torch.zeros([bs, 6])
        x_dot[:, :3] = x[:, 3:]
        x_dot[:, 3:] = u
         
        temp = z
        for i, l in enumerate(self.nn_model):
            temp = l(temp)
            if (i + 1)%3 == 0:
                x_dot += temp
                if (i+1) != len(self.nn_model):
                    temp = z
        
        out = torch.cat([x_dot, torch.zeros([bs, 3])], 1)
        #print("output shape is: ", out.shape)
        return out.unsqueeze(1)
 

class RigidHybridCascade(ODEF):
    def __init__(self):  # quadparams is only a placeholder, not actually used
        super(RigidHybridCascade, self).__init__()
        self.layers = [nn.Linear(9, 32),
                       nn.Tanh(),
                       nn.Linear(32, 6)]
        
        for i in range(len(self.layers)):
            if str(self.layers[i])[:6] == 'Linear':
                torch.nn.init.xavier_uniform_(self.layers[i].weight)
                torch.nn.init.zeros_(self.layers[i].bias)
    
        self.nn_model = nn.Sequential(*self.layers)
        self.n_old_layers = 0
    
    def cascade(self):
        self.n_old_layers = len(self.nn_model)
        new_layers = [nn.Linear(6, 20),
                      nn.Tanh(),
                      nn.Linear(20, 6)]
        for i in range(len(new_layers)):
            if str(self.layers[i])[:6] == 'Linear':
                torch.nn.init.xavier_uniform_(new_layers[i].weight)
                torch.nn.init.zeros_(new_layers[i].bias)

        self.layers += new_layers
        self.nn_model = nn.Sequential(*self.layers)
 

    def forward(self, z):
        bs,_,_ = z.size()
        z = z.squeeze(1)
        x = z[:, :6]  # 6D state
        u = z[:, 6:]  # 3D input
        
        x_dot = torch.zeros([bs, 6])
        x_dot[:, :3] = x[:, 3:]
        x_dot[:, 3:] = u
        
        x_dot_nn = self.nn_model(z)

        x_dot_hybrid = x_dot + x_dot_nn
        out = torch.cat([x_dot_hybrid, torch.zeros([bs, 3])], 1)
        #print("output shape is: ", out.shape)
        return out.unsqueeze(1)
