param BV_config{i in {1..6}};
param tf = BV_config[5];
param Nfe;
param NE = Nfe - 1;
param hi = tf / NE;
param Nobs{i in {1..2}};
set I := {0..NE};
set I1 := {1..NE};
param MO{i in I, j in {1..Nobs[1]}, k in {1..4}, n in {1..2}};
param SO{j in {1..Nobs[2]}, n in {1..2}};
param Area{i in {1..Nobs[1]}};
param VP{i in {1..10}};
param AreaVehicle = (VP[1] + VP[2]) * 2 * VP[3];

var x{i in I};
var y{i in I};
var xf{i in I};
var yf{i in I};
var xr{i in I};
var yr{i in I};
var theta{i in I};
var v{i in I};
var a{i in I};
var phy{i in I};
var w{i in I};
var egoV{i in I, j in {1..4}, k in {1..2}};

minimize cost_function:
sum{i in I1}(a[i]^2) + sum{i in I1}((a[i] - a[i-1])^2) + 10 * sum{i in I1}(phy[i]^2) + 100 * sum{i in I1}(w[i]^2);

### ODEs ###
s.t. DIFF_dxdt {i in I1}:
x[i] = x[i-1] + hi * v[i] * cos(theta[i]);
s.t. DIFF_dydt {i in I1}:
y[i] = y[i-1] + hi * v[i] * sin(theta[i]);
s.t. DIFF_dvdt {i in I1}:
v[i] = v[i-1] + hi * a[i];
s.t. DIFF_dthetadt {i in I1}:
theta[i] = theta[i-1] + hi * tan(phy[i]) * v[i] * VP[4];
s.t. DIFF_dphydt {i in I1}:
phy[i] = phy[i-1] + hi * w[i];

### Equations for vertexes A B C D ###
s.t. RELATIONSHIP_AX {i in I}:
egoV[i,1,1] = x[i] + VP[1] * cos(theta[i]) - VP[3] * sin(theta[i]);
s.t. RELATIONSHIP_BX {i in I}:
egoV[i,2,1] = x[i] + VP[1] * cos(theta[i]) + VP[3] * sin(theta[i]);
s.t. RELATIONSHIP_CX {i in I}:
egoV[i,3,1] = x[i] - VP[2] * cos(theta[i]) + VP[3] * sin(theta[i]);
s.t. RELATIONSHIP_DX {i in I}:
egoV[i,4,1] = x[i] - VP[2] * cos(theta[i]) - VP[3] * sin(theta[i]);
s.t. RELATIONSHIP_AY {i in I}:
egoV[i,1,2] = y[i] + VP[1] * sin(theta[i]) + VP[3] * cos(theta[i]);
s.t. RELATIONSHIP_BY {i in I}:
egoV[i,2,2] = y[i] + VP[1] * sin(theta[i]) - VP[3] * cos(theta[i]);
s.t. RELATIONSHIP_CY {i in I}:
egoV[i,3,2] = y[i] - VP[2] * sin(theta[i]) - VP[3] * cos(theta[i]);
s.t. RELATIONSHIP_DY {i in I}:
egoV[i,4,2] = y[i] - VP[2] * sin(theta[i]) + VP[3] * cos(theta[i]);

### Equations for xf yf xr yr ###
s.t. RELATIONSHIP_XF {i in I}:
xf[i] = x[i] + VP[10] * cos(theta[i]);
s.t. RELATIONSHIP_YF {i in I}:
yf[i] = y[i] + VP[10] * sin(theta[i]);
s.t. RELATIONSHIP_XR {i in I}:
xr[i] = x[i] + VP[9] * cos(theta[i]);
s.t. RELATIONSHIP_YR {i in I}:
yr[i] = y[i] + VP[9] * sin(theta[i]);

### Manifold constraints w.r.t. vehicle physics ###
s.t. Bonds_phy {i in I}:
-VP[7] <= phy[i] <= VP[7];
s.t. Bonds_a {i in I}:
-VP[6] <= a[i] <= VP[6];
s.t. Bonds_v {i in I}:
0 <= v[i] <= VP[5];
s.t. Bonds_w {i in I}:
-VP[8] <= w[i] <= VP[8];

### Collision avoidance 1 ###
s.t. ObsVertexOutOfABCD {i in I, j in {1..Nobs[1]}, k in {1..4}}:
0.5 * abs(MO[i,j,k,1] * egoV[i,1,2] + egoV[i,1,1] * egoV[i,2,2] + egoV[i,2,1] * MO[i,j,k,2] - MO[i,j,k,1] * egoV[i,2,2] - egoV[i,1,1] * MO[i,j,k,2] - egoV[i,2,1] * egoV[i,1,2]) + 
0.5 * abs(MO[i,j,k,1] * egoV[i,3,2] + egoV[i,3,1] * egoV[i,2,2] + egoV[i,2,1] * MO[i,j,k,2] - MO[i,j,k,1] * egoV[i,2,2] - egoV[i,3,1] * MO[i,j,k,2] - egoV[i,2,1] * egoV[i,3,2]) + 
0.5 * abs(MO[i,j,k,1] * egoV[i,3,2] + egoV[i,3,1] * egoV[i,4,2] + egoV[i,4,1] * MO[i,j,k,2] - MO[i,j,k,1] * egoV[i,4,2] - egoV[i,3,1] * MO[i,j,k,2] - egoV[i,4,1] * egoV[i,3,2]) + 
0.5 * abs(MO[i,j,k,1] * egoV[i,1,2] + egoV[i,1,1] * egoV[i,4,2] + egoV[i,4,1] * MO[i,j,k,2] - MO[i,j,k,1] * egoV[i,4,2] - egoV[i,1,1] * MO[i,j,k,2] - egoV[i,4,1] * egoV[i,1,2]) >= AreaVehicle + 2;

s.t. CarVertexOutOfObstacle {i in I, j in {1..Nobs[1]}, k in {1..4}}:
0.5 * abs(egoV[i,k,1] * MO[i,j,1,2] + MO[i,j,1,1] * MO[i,j,2,2] + MO[i,j,2,1] * egoV[i,k,2] - egoV[i,k,1] * MO[i,j,2,2] - MO[i,j,1,1] * egoV[i,k,2] - MO[i,j,2,1] * MO[i,j,1,2]) + 
0.5 * abs(egoV[i,k,1] * MO[i,j,3,2] + MO[i,j,3,1] * MO[i,j,2,2] + MO[i,j,2,1] * egoV[i,k,2] - egoV[i,k,1] * MO[i,j,2,2] - MO[i,j,3,1] * egoV[i,k,2] - MO[i,j,2,1] * MO[i,j,3,2]) + 
0.5 * abs(egoV[i,k,1] * MO[i,j,3,2] + MO[i,j,3,1] * MO[i,j,4,2] + MO[i,j,4,1] * egoV[i,k,2] - egoV[i,k,1] * MO[i,j,4,2] - MO[i,j,3,1] * egoV[i,k,2] - MO[i,j,4,1] * MO[i,j,3,2]) + 
0.5 * abs(egoV[i,k,1] * MO[i,j,1,2] + MO[i,j,1,1] * MO[i,j,4,2] + MO[i,j,4,1] * egoV[i,k,2] - egoV[i,k,1] * MO[i,j,4,2] - MO[i,j,1,1] * egoV[i,k,2] - MO[i,j,4,1] * MO[i,j,1,2]) >= Area[j] + 2;

### Collision avoidance 2 ###
s.t. VehicleItoJff {i in I}:
-2.2278 <= yf[i] <= 2.2278;
s.t. VehicleItoJrr {i in I}:
-2.2278 <= yr[i] <= 2.2278;

############# Two-Point Boundary Values #############
s.t. EQ_init_x:
x[0] = BV_config[1];
s.t. EQ_init_y:
y[0] = BV_config[2];
s.t. EQ_init_theta:
theta[0] = BV_config[3];
s.t. EQ_init_v:
v[0] = BV_config[4];
s.t. EQ_init_a:
a[0] = 0;
s.t. EQ_init_phy:
phy[0] = 0;
s.t. EQ_init_w:
w[0] = 0;

s.t. EQ_end_x:
BV_config[6] - 10 <= x[NE] <= BV_config[6] + 50;
s.t. EQ_end_y:
-2 <= y[NE] <= 2;
s.t. EQ_end_theta:
theta[NE] = 0;
s.t. EQ_end_a:
a[NE] = 0;
s.t. EQ_end_phy:
phy[NE] = 0;
s.t. EQ_end_w:
w[NE] = 0;

data;
param BV_config := include BV_config;
param Nfe := include Nfe;
param Nobs := include Nobs;
param MO := include MO;
param SO := include SO;
param Area := include Area;
param VP := include VP;