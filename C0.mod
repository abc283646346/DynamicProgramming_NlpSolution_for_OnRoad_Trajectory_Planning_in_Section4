param Nfe;
param tf;
param NE = Nfe - 1;
param hi = tf / NE;
set I := {0..NE};
set I1 := {1..NE};
param VP{i in {1..10}};

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
1;

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

data;
param Nfe := include Nfe;
param VP := include VP;