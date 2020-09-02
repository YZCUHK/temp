%--------------------------------------------------------------------------
% Model configuration
% Including geometric parameters,DH table of the 7-D
% manipulator,disturbance parameters and environmental parameters.
%--------------------------------------------------------------------------
function model=ModelConfig()

% Geometric parameters. SI unit: m, kg, s.[w] coord. upper hanging points.
% [b] coord, lower hanging points.[L] length of the rope.[mb] mass of
% cage.[ml] mass of link. [d] force arm length. [RotCoef] coefficient of
% rotor f=RotCoef*w^2, unit(N.^2)
% 这种配置不要轻易更改
    model.wpw1=[0; 0; 0];
    model.wpw2=[3; 0; 0];
    model.bpb1=[0; 0; 0];
    model.bpb2=[3; 0; 0];
    model.L=10; 
    model.d=1;
    model.RotCoef=1/200;

    cagescale=0.1;%暂时这样，如果需要更改，对应的吊篮重心要改。
    model.mb=cagescale*2500;                                               
    model.ml1=25.7;                                                  
    model.ml2=4.2;
    model.ml3=11;
    model.ml4=6;
    model.ml5=1.3;
    model.ml6=1.3;
    model.ml7=0.5;
    
% friction term b1~b13, where 3(z) and 5(theta)
% 吊篮阻尼选的比较小为了明显突出他的晃动
    const_b=20;
    model.b1=const_b*10;
    model.b2=const_b*10;
    model.b3=const_b*10;
    model.b4=const_b*10;
    model.b5=const_b*10;
    model.b6=const_b*10;
    model.b7=const_b*0.1;
    model.b8=const_b*0.01;
    model.b9=const_b*0.1;
    model.b10=const_b*0.1;
    model.b11=const_b*0.001;
    model.b12=const_b*0.001;
    model.b13=const_b*0.0001;
    
% inertia matrix of cage and link (kg*m2),at mass center, temp diagnol matrix
% 是不是对应惯量的20倍就行呢
    model.Inerb=cagescale*diag([400;2000;2000]);     
	scale=1;
    model.Inerl1=reshape(scale*diag([0.21;0.33;0.19]),9,1); 
    model.Inerl2=reshape(scale*diag([0.01;0.01;0.01]),9,1); 
    model.Inerl3=reshape(scale*diag([0.03;0.38;0.38]),9,1); 
    model.Inerl4=reshape(scale*diag([0.01;0.12;0.12]),9,1); 
    model.Inerl5=reshape(scale*diag([0.002;0.001;0.002]),9,1); 
    model.Inerl6=reshape(scale*diag([0.002;0.001;0.002]),9,1); 
    model.Inerl7=reshape(scale*diag([0.0002;0.0002;0.0003]),9,1); 

% DH table of 7-D manipulator. type: "1" for revolute,  "2" for prismatic,
% from j=1 to j=7.
    model.DH=[
% type          alpha       a           d           theta       s           gamma
%--------------------------------------------------------------------------------
	2           pi/2        0.5         0           0           0           pi/2;	
	1           pi/2        0           -0.2785     pi          0           0;    
	1           -pi/2       0           0.1405      -pi/2       0           0; 
	1           pi          0.408       0           0           0           0; 
	1           pi          0.376       -0.019      -pi/2       0           0; 
	1           -pi/2       0           0.1025      0           0           0; 
	1           pi/2        0           0.094       0           0           0   
     ];
    
% position of center of mass in each link coord,
    model.mbcenter=[1.5;-0.1;0.8;1];                                                                       
    model.mcenter=[
        0,0.4,0,1;
        0,0,0,1;
        0.2,0,0,1;
        0.19,0,0.12,1;
        0,0,0,1;
        0,0,0,1;
        0,0,-0.02,1
        ];

% environmental parameters
    model.g = 9.81;
end