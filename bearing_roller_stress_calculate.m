function [stress_outer,stress_inner,half_width_outer,half_width_inner]=bearing_roller_stress_calculate(order_bearing_interim,bearing_layout_temp,roller_modify_curve,outer_modify_curve,inner_modify_curve,Q_global,Tilt_angle_global,number_slice)
%%%参数输出 
%轴承滚动体与外圈切片应力向量(Mpa)，stress_outer
%轴承滚动体与内圈切片应力向量(Mpa)，stress_inner
%轴承滚动体与外圈切片实际接触半宽向量(mm)，half_width_outer
%轴承滚动体与内圈切片实际接触半宽向量(mm)，half_width_inner
%%%参数输入
%轴承（除概念轴承外）序号，order_bearing_interim
%轴系的轴承（除概念轴承外）装配参数矩阵，bearing_layout_temp
%球轴承/滚子轴承 [1 轴承序号 2 与其内圈联结轴的序号 3 轴承内圈节点序号 4 轴承中心距内圈联结轴左端距离（mm） 5 与其外圈联结轴的序号 6 轴承外圈节点序号 7 轴承中心距外圈联结轴左端距离（mm） 8 轴承额定动载荷(N)，capacity_dynamic 9 轴承额定静载荷(N)，capacity_static 10-12 空 13 滚动体列数，bearing_element_column 14 轴承类型，bearing_type 15 轴承质量(kg)，bearing_mass 16 轴承宽度(mm)，bearing_width 17 轴承内圈(mm)，bearing_inner 18 轴承外圈(mm)，bearing_outer 19 球轴承初始接触角/滚子轴承接触角(rad)，contact_angle_initial 20 球轴承内圈沟曲率，bearing_fi 21 球轴承外圈沟曲率，bearing_fe 22 滚动体个数，bearing_element_number 23 滚动体直径/锥轴承滚动体中径(mm)，bearing_element_diameter 24 锥轴承滚动体锥角(rad)，taper_angle 25 滚子轴承滚动体长度(mm)，bearing_element_length 26 滚子轴承圆角半径(mm)，bearing_element_fillet 27 轴承径向游隙(mm)，bearing_radial_clearance 28 轴承轴向游隙(mm)，bearing_axial_clearance 29 内圈挡边位置（0表示左边，1表示右边），bearing_rib_position 30 圆柱滚子内圈左边是否带档边，bearing_rib_inner_left 31 圆柱滚子内圈右边是否带档边，bearing_rib_inner_right 32 圆柱滚子外圈左边是否带档边，bearing_rib_outer_left 33 圆柱滚子外圈右边是否带档边，bearing_rib_outer_right 34 滚动体修形类型，type_roller_modi 35 外圈修形类型，type_outer_modi 36 内圈修形类型，type_inner_modi 37 修形曲线数据点个数 38 轴承外圈弹性模量(Mpa)，E_outer 39 轴承外圈泊松比，v_outer 40 轴承滚动体弹性模量(Mpa),E_element 41 轴承滚动体泊松比，v_element 42 轴承内圈弹性模量(Mpa)，E_inner 43 轴承内圈泊松比，v_inner 44 滚动体切片个数]
%滚子轴承滚动体修形曲线矩阵，roller_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 滚动体修形曲线数据点个数 6 修形曲线数据]
%滚子轴承外圈修形曲线矩阵，outer_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 外圈修形曲线数据点个数 6 修形曲线数据]
%滚子轴承内圈修形曲线矩阵，inner_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 内圈修形曲线数据点个数 6 修形曲线数据]
%轴承全局坐标系的滚动体法向载荷，Q_global
%轴承全局坐标系的滚动体倾斜角，Tilt_angle_global
%轴承滚动体切片个数，number_slice
%%%程序开始
%%%记录序号为order_bearing_interim轴承（除概念轴承外）在轴系的轴承（除概念轴承外）装配参数矩阵的行数
global a_cnt;
global D_cnt
global outloop_cnt;
global time_cnt;
id_bearing_layout=(bearing_layout_temp(:,1)==order_bearing_interim);
%%%记录序号为order_bearing_interim轴承（除概念轴承外）的滚动体个数
number_roller=bearing_layout_temp(id_bearing_layout,22); %滚动体个数
%%%定义轴承滚动体与外圈切片应力向量(Mpa)，stress_outer
stress_outer=zeros(number_slice,number_roller);
%%%定义轴承滚动体与内圈切片应力向量(Mpa)，stress_inner
stress_inner=zeros(number_slice,number_roller);
%%%定义轴承滚动体与外圈切片实际接触半宽向量(mm)，half_width_outer
half_width_outer=zeros(number_slice,number_roller);
%%%定义轴承滚动体与内圈切片实际接触半宽向量(mm)，half_width_inner
half_width_inner=zeros(number_slice,number_roller);
Q_global=roundn(Q_global,0);
tic;
t0=clock;
D_cnt=0;
a_cnt=0;
time_cnt=0
outloop_cnt=0;
for i=1:number_roller  %滚动体个数
    if abs(Q_global(i,1))>1
        %%%计算各个滚动体的切片与外圈/内圈应力向量(Mpa)，stress_outer/stress_inner；与外圈/内圈实际接触半宽向量(mm)，half_width_outer/half_width_inner
        [stress_outer(:,i),stress_inner(:,i),half_width_outer(:,i),half_width_inner(:,i)]=Rollerbearing_stress_sub(order_bearing_interim,bearing_layout_temp,roller_modify_curve,outer_modify_curve,inner_modify_curve,number_slice,Tilt_angle_global(i,1),Q_global(i,1));
    end
end
number_roller
D_cnt
a_cnt
outloop_cnt
time_cnt
avg_time=time_cnt/D_cnt
disp(['程序总运行时间：',num2str(etime(clock,t0))])
%%%程序结束 

%%%计算各个滚动体的切片与外圈/内圈应力向量(Mpa)，stress_outer/stress_inner；与外圈/内圈实际接触半宽向量(mm)，half_width_outer/half_width_inner
function [stress_outer,stress_inner,a_i_outer,a_i_inner]=Rollerbearing_stress_sub(order_bearing_interim,bearing_layout_temp,roller_modify_curve,outer_modify_curve,inner_modify_curve,number_slice,tilt_angle_single,force)
%%%参数输出
%轴承各个滚动体与外圈切片应力向量(Mpa)，stress_outer
%轴承各个滚动体与内圈切片应力向量(Mpa)，stress_inner
%轴承各个滚动体与外圈切片实际接触半宽向量(mm)，a_i_outer
%轴承各个滚动体与内圈切片实际接触半宽向量(mm)，a_i_inner
%%%参数输入
%轴承（除概念轴承外）序号，order_bearing_interim
%轴系的轴承（除概念轴承外）装配参数矩阵，bearing_layout_temp
%球轴承/滚子轴承 [1 轴承序号 2 与其内圈联结轴的序号 3 轴承内圈节点序号 4 轴承中心距内圈联结轴左端距离（mm） 5 与其外圈联结轴的序号 6 轴承外圈节点序号 7 轴承中心距外圈联结轴左端距离（mm） 8 轴承额定动载荷(N)，capacity_dynamic 9 轴承额定静载荷(N)，capacity_static 10-12 空 13 滚动体列数，bearing_element_column 14 轴承类型，bearing_type 15 轴承质量(kg)，bearing_mass 16 轴承宽度(mm)，bearing_width 17 轴承内圈(mm)，bearing_inner 18 轴承外圈(mm)，bearing_outer 19 球轴承初始接触角/滚子轴承接触角(rad)，contact_angle_initial 20 球轴承内圈沟曲率，bearing_fi 21 球轴承外圈沟曲率，bearing_fe 22 滚动体个数，bearing_element_number 23 滚动体直径/锥轴承滚动体中径(mm)，bearing_element_diameter 24 锥轴承滚动体锥角(rad)，taper_angle 25 滚子轴承滚动体长度(mm)，bearing_element_length 26 滚子轴承圆角半径(mm)，bearing_element_fillet 27 轴承径向游隙(mm)，bearing_radial_clearance 28 轴承轴向游隙(mm)，bearing_axial_clearance 29 内圈挡边位置（0表示左边，1表示右边），bearing_rib_position 30 圆柱滚子内圈左边是否带档边，bearing_rib_inner_left 31 圆柱滚子内圈右边是否带档边，bearing_rib_inner_right 32 圆柱滚子外圈左边是否带档边，bearing_rib_outer_left 33 圆柱滚子外圈右边是否带档边，bearing_rib_outer_right 34 滚动体修形类型，type_roller_modi 35 外圈修形类型，type_outer_modi 36 内圈修形类型，type_inner_modi 37 修形曲线数据点个数 38 轴承外圈弹性模量(Mpa)，E_outer 39 轴承外圈泊松比，v_outer 40 轴承滚动体弹性模量(Mpa),E_element 41 轴承滚动体泊松比，v_element 42 轴承内圈弹性模量(Mpa)，E_inner 43 轴承内圈泊松比，v_inner 44 滚动体切片个数]
%滚子轴承滚动体修形曲线矩阵，roller_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 滚动体修形曲线数据点个数 6 修形曲线数据]
%滚子轴承外圈修形曲线矩阵，outer_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 外圈修形曲线数据点个数 6 修形曲线数据]
%滚子轴承内圈修形曲线矩阵，inner_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 内圈修形曲线数据点个数 6 修形曲线数据]
%轴承滚动体切片个数，number_slice
%轴承全局坐标系的单个滚动体倾斜角，tilt_angle_single
%轴承全局坐标系的单个滚动体法向载荷，force
%%%程序开始
%%%记录序号为order_bearing_interim轴承（除概念轴承外）在轴系的轴承（除概念轴承外）装配参数矩阵的行数，id_bearing_layout
id_bearing_layout=(bearing_layout_temp(:,1)==order_bearing_interim);
%%%提取轴承结构尺寸
d_inner=bearing_layout_temp(id_bearing_layout,17);  %轴承内径，mm
D_outer=bearing_layout_temp(id_bearing_layout,18);  %轴承外径，mm
alpha_e=bearing_layout_temp(id_bearing_layout,19);   %滚子轴承接触角（外圈接触角），rad
Db=bearing_layout_temp(id_bearing_layout,23); %滚动体中径，mm
taper_angle=bearing_layout_temp(id_bearing_layout,24); %滚动体锥角，rad
L_roller_initial=bearing_layout_temp(id_bearing_layout,25);  %滚子实际长度，mm
L_roller_initial=abs(L_roller_initial);
r_roller=bearing_layout_temp(id_bearing_layout,26); %滚子两端圆角半径，mm
type_roller_modi=bearing_layout_temp(id_bearing_layout,34); % 轴承滚子是否修形，0代表无修形，1代表两端抛物线修形，2代表对数曲线修形，3代表自定义修形量
type_outer_modi=bearing_layout_temp(id_bearing_layout,35); % 轴承外圈是否修形，0代表无修形，1代表两端抛物线修形，2代表对数曲线修形，3代表自定义修形量
type_inner_modi=bearing_layout_temp(id_bearing_layout,36); % 轴承内圈是否修形，0代表无修形，1代表两端抛物线修形，2代表对数曲线修形，3代表自定义修形量
E_outer=bearing_layout_temp(id_bearing_layout,38); %外圈材料弹性模量，MPa
u_outer=bearing_layout_temp(id_bearing_layout,39); %外圈材料泊松比
E_roller=bearing_layout_temp(id_bearing_layout,40); %滚动体材料弹性模量，MPa
u_roller=bearing_layout_temp(id_bearing_layout,41); %滚动体材料泊松比
E_inner=bearing_layout_temp(id_bearing_layout,42); %内圈材料弹性模量，MPa
u_inner=bearing_layout_temp(id_bearing_layout,43); %内圈材料泊松比
%计算滚动体实际长度（mm），L_roller
L_roller=L_roller_initial-2*r_roller;
%计算滚动体与外圈接触综合弹性模量（Mpa），E_roller_outer
E_roller_outer=((1-u_roller^2)/E_roller+(1-u_outer^2)/E_outer)^(-1);
%计算滚动体与内圈接触综合弹性模量（Mpa），E_roller_inner
E_roller_inner=((1-u_roller^2)/E_roller+(1-u_inner^2)/E_inner)^(-1);
%%%计算按切片个数计算滚动体相对于外圈的综合倾斜曲线
number_slice=ceil(abs(number_slice));
if mod(number_slice,2)==1
    number_slice=number_slice+1;
end
%计算滚动体切片半宽（mm），hj
hj=0.5*L_roller/number_slice;
%沿滚动体切片中点位置向量（中点为零点）（mm），yyj
yyj=zeros(number_slice,1);
%拟合滚动体自定义修形曲线，p_roller，[2次项系数 1次项系数 常系数]
if type_roller_modi==3  %滚动体自定义修形
    %提取滚动体自定义修形曲线的数据，（mm），profile_modification_roller
    %记录序号为order_bearing_interim轴承（除概念轴承外）在滚子轴承滚动体修形曲线矩阵的行数，id_roller_modify
    id_roller_modify=(roller_modify_curve(:,1)==order_bearing_interim);
    %滚子轴承滚动体修形曲线矩阵，roller_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 滚动体修形曲线数据点个数 6 修形曲线数据]
    roller_curve_point=roller_modify_curve(id_roller_modify,5); %修形数据点数
    %记录序号为order_bearing的轴承的滚动体修形曲线参数，profile_modification_roller
    profile_modification_roller=roller_modify_curve(id_roller_modify,6:roller_curve_point+5); %滚动体修形量，mm，注意每个轴承的修形参数输入为行向量
    %拟合滚动体自定义修形曲线，p_roller，[2次项系数 1次项系数 常系数]
    xxj=zeros(1,roller_curve_point); %滚动体修形数据点数位置向量（滚动体中点为零点）（mm），类似yyj
    mm=0;
    for jj=(-roller_curve_point/2):(roller_curve_point/2-1)
        mm=mm+1;
        xxj(1,mm)=(2*jj+1)*0.5*L_roller/roller_curve_point;
    end
    p_roller=polyfit(xxj,profile_modification_roller,2);  %算按切片拟合滚动体自定义修形曲线，p_roller，[2次项系数 1次项系数 常系数]
end
%拟合外圈自定义修形曲线，p_outer，[2次项系数 1次项系数 常系数]
if type_outer_modi==3  %外圈自定义修形
    %提取外圈自定义修形曲线的数据，（mm），profile_modification_outer
    %记录序号为order_bearing的轴承在滚子轴承外圈修形曲线矩阵的行数
    id_outer_modify=(outer_modify_curve(:,1)==order_bearing_interim);
    %记录序号为order_bearing的轴承的外圈修形数据点，outer_curve_point
    outer_curve_point=outer_modify_curve(id_outer_modify,5);
    %记录序号为order_bearing的轴承的外圈修形曲线参数，profile_modification_outer
    profile_modification_outer=outer_modify_curve(id_outer_modify,6:outer_curve_point+5);
    %拟合外圈自定义修形曲线，p_outer，[2次项系数 1次项系数 常系数]
    xxj=zeros(1,outer_curve_point); %外圈修形数据点数位置向量（滚动体中点为零点）（mm），类似yyj
    mm=0;
    for jj=(-outer_curve_point/2):(outer_curve_point/2-1)
        mm=mm+1;
        xxj(1,mm)=(2*jj+1)*0.5*L_roller/outer_curve_point;
    end
    p_outer=polyfit(xxj,profile_modification_outer,2);  %按切片拟合外圈自定义修形曲线，p_outer，[2次项系数 1次项系数 常系数]
end
%拟合内圈自定义修形曲线，p_inner，[2次项系数 1次项系数 常系数]
if type_inner_modi==3  %内圈自定义修形
    %提取内圈自定义修形曲线的数据，（mm），profile_modification_inner
    %记录序号为order_bearing的轴承在滚子轴承内圈修形曲线矩阵的行数
    id_inner_modify=(inner_modify_curve(:,1)==order_bearing_interim);
    %记录序号为order_bearing的轴承的内圈修形数据点，inner_curve_point
    inner_curve_point=inner_modify_curve(id_inner_modify,5);
    %记录序号为order_bearing的轴承的内圈修形曲线参数，profile_modification_inner
    profile_modification_inner=inner_modify_curve(id_inner_modify,6:inner_curve_point+5);
    %拟合内圈自定义修形曲线，p_inner，[2次项系数 1次项系数 常系数]
    xxj=zeros(1,inner_curve_point); %内圈修形数据点数位置向量（滚动体中点为零点）（mm），类似yyj
    mm=0;
    for jj=(-inner_curve_point/2):(inner_curve_point/2-1)
        mm=mm+1;
        xxj(1,mm)=(2*jj+1)*0.5*L_roller/inner_curve_point;
    end
    p_inner=polyfit(xxj,profile_modification_inner,2);  %按切片拟合内圈自定义修形曲线，p_inner，[2次项系数 1次项系数 常系数]
end
%记录滚动体相对于外圈的倾斜角（rad），tilt_angle
if bearing_layout_temp(id_bearing_layout,29)==1  %轴承内圈挡边在右边
    tilt_angle=-1*tilt_angle_single;
else  %轴承内圈挡边在左边
    tilt_angle=tilt_angle_single;
end
%定义滚动体相对于外圈/内圈基于切片的倾斜曲线向量
z_roller=zeros(number_slice,1);
%按切片数计算滚子轴承滚动体相对于外圈/内圈的倾斜曲线
if type_roller_modi==3  %滚动体自定义修形
    relief_roller=zeros(number_slice,1);
end
kk=0;  %滚动体切片计数
for ii=(-number_slice/2):(number_slice/2-1)
    kk=kk+1;
    yyj(kk,1)=(2*ii+1)*hj;  %切片中点位置向量（滚动体中点为零点）（mm），yyj
    if type_roller_modi==0  %滚动体无修形
        z_roller(kk,1)=(0.5*L_roller+yyj(kk,1))*tan(tilt_angle);
    elseif type_roller_modi==1  %滚动体两端抛物线修形
        %滚子轴承滚动体修形曲线矩阵，roller_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 滚动体修形曲线数据点个数 6 修形曲线数据]
        un_modi_line=roller_modify_curve(id_roller_modify,3); %两端抛物线修形，未修形长度，mm
        modi_radius=roller_modify_curve(id_roller_modify,4);  %两端抛物线修形，修形圆弧曲率半径，mm        
        if abs(yyj(kk,1))>0.5*un_modi_line
            z_roller(kk,1)=(yyj(kk,1)^2-(0.5*un_modi_line)^2)/(2*modi_radius)+(0.5*L_roller+yyj(kk,1))*tan(tilt_angle);
        else
            z_roller(kk,1)=(0.5*L_roller+yyj(kk,1))*tan(tilt_angle);
        end
    elseif type_roller_modi==2  %滚动体对数修形
        %滚子轴承滚动体修形曲线矩阵，roller_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 滚动体修形曲线数据点个数 6 修形曲线数据]
        modi_Q=roller_modify_curve(id_roller_modify,3); %对数修形，修形载荷，N
        z_roller(kk,1)=modi_Q*log(1/(1-(2*yyj(kk,1)/L_roller)^2))/(pi*E_roller_outer*L_roller)+(0.5*L_roller+yyj(kk,1))*tan(tilt_angle);
    elseif type_roller_modi==3  %滚动体自定义修形
        relief_roller(kk,1)=(p_roller(1)* yyj(kk,1)^2+p_roller(2)* yyj(kk,1)+p_roller(3))*((p_roller(1)* yyj(kk,1)^2+p_roller(2)* yyj(kk,1)+p_roller(3))>0);
        z_roller(kk,1)=relief_roller(kk,1)+(0.5*L_roller+yyj(kk,1))*tan(tilt_angle);
    end
end
%按切片个数计算滚子轴承外圈的修形曲线
if type_outer_modi==3  %外圈自定义修形
    relief_outer=zeros(number_slice,1);
end
kk=0;  %外圈切片计数
%计算外圈基于切片的修形曲线向量
z_outer=zeros(number_slice,1);
if type_outer_modi~=0  %外圈修形
    for ii=(-number_slice/2):(number_slice/2-1)  %外圈修形
        kk=kk+1;
        yyj(kk,1)=(2*ii+1)*hj;  %切片中点位置向量（滚动体中点为零点）（mm），yyj
        if type_outer_modi==1  %外圈两端抛物线修形
            %滚子轴承外圈修形曲线矩阵，outer_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 外圈修形曲线数据点个数 6 修形曲线数据]
            un_modi_line=outer_modify_curve(id_outer_modify,3); %两端抛物线修形，未修形长度，mm
            modi_radius=outer_modify_curve(id_outer_modify,4);  %两端抛物线修形，修形圆弧曲率半径，mm
            if abs(yyj(kk,1))>0.5*un_modi_line
                z_outer(kk,1)=(yyj(kk,1)^2-(0.5*un_modi_line)^2)/(2*modi_radius);
            end
        elseif type_outer_modi==2  %外圈对数修形
            %滚子轴承外圈修形曲线矩阵，outer_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 外圈修形曲线数据点个数 6 修形曲线数据]
            modi_Q=outer_modify_curve(id_outer_modify,3); %对数修形，修形载荷，N
            z_outer(kk,1)=modi_Q*log(1/(1-(2*yyj(kk,1)/L_roller)^2))/(pi*E_roller_outer*L_roller);
        elseif type_outer_modi==3  %外圈自定义修形
            relief_outer(kk,1)=(p_outer(1)* yyj(kk,1)^2+p_outer(2)* yyj(kk,1)+p_outer(3))*((p_outer(1)* yyj(kk,1)^2+p_outer(2)* yyj(kk,1)+p_outer(3))>0);
            z_outer(kk,1)=relief_outer(kk,1);
        end
    end
end
%按切片个数计算滚子轴承内圈的修形曲线
if type_inner_modi==3  %内圈自定义修形
    relief_inner=zeros(number_slice,1);
end
kk=0;  %内圈切片计数
%计算内圈基于切片的修形曲线向量
z_inner=zeros(number_slice,1);
if type_inner_modi~=0  %内圈修形
    for ii=(-number_slice/2):(number_slice/2-1)
        kk=kk+1;
        yyj(kk,1)=(2*ii+1)*hj;  %切片中点位置向量（滚动体中点为零点）（mm），yyj
        if type_inner_modi==1  %内圈两端抛物线修形
            %滚子轴承内圈修形曲线矩阵，inner_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 内圈修形曲线数据点个数 6 修形曲线数据]
            un_modi_line=inner_modify_curve(id_inner_modify,3); %两端抛物线修形，未修形长度，mm
            modi_radius=inner_modify_curve(id_inner_modify,4);  %两端抛物线修形，修形圆弧曲率半径，mm
            if abs(yyj(kk,1))>0.5*un_modi_line
                z_inner(kk,1)=(yyj(kk,1)^2-(0.5*un_modi_line)^2)/(2*modi_radius);
            end
        elseif type_inner_modi==2  %内圈对数修形
            %滚子轴承内圈修形曲线矩阵，inner_modify_curve，[1 轴承序号 2 修形类型 3 抛物线修形未修形长度（mm）/对数修形参考载荷（N） 4 抛物线修形两端圆弧曲率半径（mm） 5 内圈修形曲线数据点个数 6 修形曲线数据]
            modi_Q=inner_modify_curve(id_inner_modify,3); %对数修形，修形载荷，N
            z_inner(kk,1)=modi_Q*log(1/(1-(2*yyj(kk,1)/L_roller)^2))/(pi*E_roller_inner*L_roller);
        elseif type_inner_modi==3  %内圈自定义修形
            relief_inner(kk,1)=(p_inner(1)* yyj(kk,1)^2+p_inner(2)* yyj(kk,1)+p_inner(3))*((p_inner(1)* yyj(kk,1)^2+p_inner(2)* yyj(kk,1)+p_inner(3))>0);
            z_inner(kk,1)=relief_inner(kk,1);
        end
    end
end
%计算按切片个数计算滚动体相对于外圈的综合倾斜曲线
z_total_outer=z_roller+z_outer;
z_total_outer=z_total_outer-min(z_total_outer);
%计算按切片个数计算滚动体相对于内圈的综合倾斜曲线
z_total_inner=z_roller+z_inner;
z_total_inner=z_total_inner-min(z_total_inner);
%%%计算滚子轴承滚动体与外圈的综合半径向量（mm），R_total_outer
r_element=0.5*Db;  %滚动体半径(mm)，r_element
r_element=abs(r_element);
taper_semi_angle=0.5*taper_angle; %滚子半锥角，rad
taper_semi_angle=abs(taper_semi_angle);
%定义切片中点位置向量（左端为零点）（mm），lj
lj=zeros(number_slice,1);
%计算滚子轴承的中径（mm），dm
dm=(d_inner+D_outer)/2;
%%%计算外圈法向接触半径的中值及范围，R_outer_mid，R_outer_min，R_outer_max
R_outer_mid=(0.5*dm+r_element*cos(alpha_e-taper_semi_angle))/cos(alpha_e);
R_outer_min=R_outer_mid-0.5*L_roller*sin(taper_semi_angle);
R_outer_max=R_outer_mid+0.5*L_roller*sin(taper_semi_angle);
%%%计算内圈法向接触半径的中值及范围，R_inner_mid，R_inner_min，R_inner_max
R_inner_mid=(0.5*dm-r_element*cos(alpha_e-taper_semi_angle))/cos(alpha_e-2*taper_semi_angle);
R_inner_min=R_inner_mid-0.5*L_roller*sin(taper_semi_angle);
R_inner_max=R_inner_mid+0.5*L_roller*sin(taper_semi_angle);
%%%计算滚动体法向接触半径的中值及范围，R_roller_mid，R_roller_min，R_roller_max
R_roller_mid=r_element/cos(taper_semi_angle);
R_roller_min=R_roller_mid-0.5*L_roller*sin(taper_semi_angle);
R_roller_max=R_roller_mid+0.5*L_roller*sin(taper_semi_angle);
%%%定义外圈法向接触半径向量（mm），R_outer
R_outer=R_outer_mid*ones(number_slice,1);
%%%定义内圈法向接触半径向量（mm），R_inner
R_inner=R_inner_mid*ones(number_slice,1);
%%%定义滚动体法向接触半径向量（mm），R_roller
R_roller=R_roller_mid*ones(number_slice,1);
%%%计算滚子轴承滚动体与外圈/内圈的综合半径向量（mm），R_total_outer/R_total_inner
if taper_semi_angle~=0  %轴承类型为圆锥轴承
    for i=1:number_slice;
        %计算各切片中点的位置
        lj(i,1)=hj+(i-1)*2*hj;
        %计算各切片中点对应的滚动体、外圈的法向接触半径
        if bearing_layout_temp(id_bearing_layout,29)==0  %轴承内圈挡边在右边
            %计算各切片中点对应的外圈的法向接触半径（mm），R_outer
            R_outer(i,1)=(1-lj(i,1)/L_roller)*R_outer_min+(lj(i,1)/L_roller)*R_outer_max;
            %计算各切片中点对应的内圈的法向接触半径（mm），R_inner
            R_inner(i,1)=(1-lj(i,1)/L_roller)*R_inner_min+(lj(i,1)/L_roller)*R_inner_max;
            %计算各切片中点对应的滚动体的法向接触半径（mm），R_roller
            R_roller(i,1)=(1-lj(i,1)/L_roller)*R_roller_min+(lj(i,1)/L_roller)*R_roller_max;
        else  %轴承内圈挡边在左边
            %计算各切片中点对应的外圈的法向接触半径（mm），R_outer
            R_outer(i,1)=(1-lj(i,1)/L_roller)*R_outer_max+(lj(i,1)/L_roller)*R_outer_min;
            %计算各切片中点对应的内圈的法向接触半径（mm），R_inner
            R_inner(i,1)=(1-lj(i,1)/L_roller)*R_inner_max+(lj(i,1)/L_roller)*R_inner_min;
            %计算各切片中点对应的滚动体的法向接触半径（mm），R_roller
            R_roller(i,1)=(1-lj(i,1)/L_roller)*R_roller_max+(lj(i,1)/L_roller)*R_roller_min;
        end
    end
end
%计算滚子轴承滚动体与外圈的综合半径向量（mm），R_total_outer
R_total_outer=abs(((R_roller.^-1)-(R_outer.^-1)).^-1);
%计算滚子轴承滚动体与内圈的综合半径向量（mm），R_total_inner
R_total_inner=abs(((R_roller.^-1)+(R_inner.^-1)).^-1);
%%%计算给定法向力的滚动体与外圈应力与实际接触半宽
force=abs(force);
%定义收敛容差，tolerance
tolerance=0.035;
%计算滚动体的初始法向位移，delta_initial
ratio_r=R_roller_mid/R_outer_mid;
delta_initial=4.83*(10^-5)*force^0.9*(1-ratio_r)^0.1/(L_roller^0.74*(2*R_roller_mid)^0.1);
%定义滚动体相对于外圈的法向位移
delta_outer=delta_initial;
%定义滚动体与外圈初始法向位移迭代变化值（mm），delta_interval_outer
delta_interval_outer=delta_initial/10;
%计算给定初始法向位移的滚动体与外圈整体载荷、滚动体应力向量、接触半宽向量
%计算给定滚动体相对于外圈的法向位移初始状态的切片受载变形向量，S_outer
S_outer=(delta_outer-z_total_outer).*(delta_outer-z_total_outer>0);   %切片受载变形向量（mm）
while (sum(S_outer)==0)
    delta_outer=delta_outer+delta_interval_outer;
    S_outer=(delta_outer-z_total_outer).*(delta_outer-z_total_outer>0);   %切片受载变形向量（mm）
end
[contact_force_outer,contact_stress_outer,a_i_outer,a_i_outer_first,a_i_outer_last,mark_outer]=linear_contact_inter(R_total_outer,hj,yyj,z_total_outer,number_slice,E_roller_outer,delta_outer);
%滚动体与外圈接触法向力（N），contact_force_outer
%滚动体与外圈切片接触应力向量（MPa），contact_stress_outer
%滚动体与外圈切片实际接触半宽向量(mm)，a_i_outer
%滚动体与外圈实际接触区最左边切片的序号，a_i_outer_first
%滚动体与外圈实际接触区最右切片的序号，a_i_outer_last
if mark_outer==1  %滚动体与外圈初始法向位移滚动体切片接触状态，mark，0表示有切片接触，1表示无切片接触
    stress_outer=contact_stress_outer;
else  %滚动体与外圈初始法向位移滚动体切片有接触状态
    kk=0;  %法向位移迭代变化值记数
    if contact_force_outer<force  %滚动体与外圈初始法向位移的滚动体整体法向力小于给定的法向力
        %计算滚动体与外圈 给定法向力与计算法向力的误差
        force_dif=(contact_force_outer-force)/contact_force_outer;
        while abs(force_dif)>tolerance
            %计算滚动体与外圈新状态的法向位移
            delta_outer=delta_outer+delta_interval_outer;
            kk=kk+1;   %法向位移迭代变化值记数
            if kk>=5
                delta_interval_outer=delta_interval_outer*2;
                kk=0;  %法向位移迭代变化值记数
            end
            %滚动体与外圈给定法向位移计算滚动体接触法向力、应力向量、接触半宽
            [contact_force_outer,contact_stress_outer,a_i_outer,a_i_outer_first,a_i_outer_last]=linear_contact_inter(R_total_outer,hj,yyj,z_total_outer,number_slice,E_roller_outer,delta_outer);
            %计算滚动体与外圈给定法向力与计算法向力的误差
            force_dif=(contact_force_outer-force)/contact_force_outer;
            if contact_force_outer>force  %滚动体与外圈整体法向力大于给定的法向力
                if abs(force_dif)<=tolerance
                    break;
                end
                %滚动体与外圈法向位移返回至上一步迭代的法向位移
                delta_outer=delta_outer-delta_interval_outer;
                %滚动体与外圈初始法向位移迭代变化值减小
                delta_interval_outer=delta_interval_outer/10;
                kk=0;  %法向位移迭代变化值记数
            end
            %判断法各位移迭代变化值小于给定误差
            if delta_interval_outer<10^-6
                break;
            end
        end
    elseif contact_force_outer>force  %滚动体与外圈初始法向位移的滚动体整体法向力大于给定的法向力
        %计算滚动体与外圈给定法向力与计算法向力的误差
        force_dif=(contact_force_outer-force)/force;
        while abs(force_dif)>tolerance
            %计算滚动体与外圈新状态的法向位移
            delta_outer=delta_outer-delta_interval_outer;
            if delta_outer<=0
                delta_interval_outer=delta_interval_outer/10;
                continue;
            end
            kk=kk+1;   %法向位移迭代变化值记数
            if kk>=5
                delta_interval_outer=delta_interval_outer*2;
                kk=0;  %法向位移迭代变化值记数
            end
            %滚动体与外圈给定法向位移计算滚动体接触法向力、应力向量、接触半宽
            [contact_force_outer,contact_stress_outer,a_i_outer,a_i_outer_first,a_i_outer_last]=linear_contact_inter(R_total_outer,hj,yyj,z_total_outer,number_slice,E_roller_outer,delta_outer);
            %计算滚动体与外圈给定法向力与计算法向力的误差
            force_dif=(contact_force_outer-force)/force;
            if contact_force_outer<force  %滚动体与外圈整体法向力小于给定的法向力
                if abs(force_dif)<=tolerance
                    break;
                end
                %滚动体与外圈法向位移返回至上一步迭代的法向位移
                delta_outer=delta_outer+delta_interval_outer;
                %初始法向位移迭代变化值减小
                delta_interval_outer=delta_interval_outer/10;
                kk=0;  %法向位移迭代变化值记数
            end
            %判断法各位移迭代变化值小于给定误差
           % if delta_interval_outer<10^-6
               % break;
           % end
        end
    end
    %记录滚动体与外圈切片接触应力向量（Mpa）,stress_outer
    stress_outer=zeros(number_slice,1);
    stress_outer(a_i_outer_first:a_i_outer_last,1)=contact_stress_outer;
end
%%%计算给定法向力的滚动体与内圈应力与实际接触半宽
%定义滚动体与内圈初始法向位移，delta_inner
delta_inner=delta_initial;
%定义滚动体与内圈初始法向位移迭代变化值（mm），delta_interval_inner
delta_interval_inner=delta_initial/10;
%计算给定滚动体相对于内圈的法向位移初始状态的切片受载变形向量，S_inner
S_inner=(delta_inner-z_total_inner).*(delta_inner-z_total_inner>0);   %切片受载变形向量（mm）
while (sum(S_inner)==0)
    delta_inner=delta_inner+delta_interval_inner;
    S_inner=(delta_inner-z_total_inner).*(delta_inner-z_total_inner>0);   %切片受载变形向量（mm）
end
%计算给定初始法向位移的滚动体与内圈整体载荷、滚动体应力向量、接触半宽向量
[contact_force_inner,contact_stress_inner,a_i_inner,a_i_inner_first,a_i_inner_last,mark_inner]=linear_contact_inter(R_total_inner,hj,yyj,z_total_inner,number_slice,E_roller_inner,delta_inner);
%滚动体与内圈接触法向力（N），contact_force_inner
%滚动体与内圈切片接触应力向量（MPa），contact_stress_inner
%滚动体与内圈切片实际接触半宽向量(mm)，a_i_inner
%滚动体与内圈实际接触区最左边切片的序号，a_i_inner_first
%滚动体与内圈实际接触区最右切片的序号，a_i_inner_last
if mark_inner==1  %滚动体与内圈初始法向位移滚动体切片接触状态，mark，0表示有切片接触，1表示无切片接触
    stress_inner=contact_stress_inner;
else  %滚动体与内圈初始法向位移滚动体切片有接触状态
    kk=0;  %法向位移迭代变化值记数
    if contact_force_inner<force  %滚动体与内圈初始法向位移的滚动体整体法向力小于给定的法向力
        %计算滚动体与内圈给定法向力与计算法向力的误差
        force_dif=(contact_force_inner-force)/contact_force_inner;
        while abs(force_dif)>tolerance
            %计算滚动体与内圈新状态的法向位移
            delta_inner=delta_inner+delta_interval_inner;
            kk=kk+1;   %法向位移迭代变化值记数
            if kk>=5
                delta_interval_outer=delta_interval_outer*2;
                kk=0;  %法向位移迭代变化值记数
            end
            %滚动体与内圈给定法向位移计算滚动体接触法向力、应力向量、接触半宽
            [contact_force_inner,contact_stress_inner,a_i_inner,a_i_inner_first,a_i_inner_last]=linear_contact_inter(R_total_inner,hj,yyj,z_total_inner,number_slice,E_roller_inner,delta_inner);
            %计算滚动体与内圈给定法向力与计算法向力的误差
            force_dif=(contact_force_inner-force)/contact_force_inner;
            if contact_force_inner>force  %滚动体与内圈整体法向力大于给定的法向力
                %滚动体与内圈法向位移返回至上一步迭代的法向位移
                delta_inner=delta_inner-delta_interval_inner; 
                %滚动体与内圈初始法向位移迭代变化值减半
                delta_interval_inner=delta_interval_inner/10;
                kk=0;  %法向位移迭代变化值记数
            end
            %判断法各位移迭代变化值小于给定误差
            if delta_interval_inner<10^-6
                break;
            end
        end
    elseif contact_force_inner>force  %滚动体与内圈初始法向位移的滚动体整体法向力大于给定的法向力
        %计算滚动体与内圈给定法向力与计算法向力的误差
        force_dif=(contact_force_inner-force)/force;
        while abs(force_dif)>tolerance
            %计算滚动体与内圈新状态的法向位移
            delta_inner=delta_inner-delta_interval_inner;
            kk=kk+1;   %法向位移迭代变化值记数
            if kk>=5
                delta_interval_outer=delta_interval_outer*2;
                kk=0;  %法向位移迭代变化值记数
            end
            %滚动体与内圈给定法向位移计算滚动体接触法向力、应力向量、接触半宽
            [contact_force_inner,contact_stress_inner,a_i_inner,a_i_inner_first,a_i_inner_last]=linear_contact_inter(R_total_inner,hj,yyj,z_total_inner,number_slice,E_roller_inner,delta_inner);
            %计算滚动体与内圈给定法向力与计算法向力的误差
            force_dif=(contact_force_inner-force)/force;
            if contact_force_inner<force  %滚动体与内圈整体法向力小于给定的法向力
                %滚动体与内圈法向位移返回至上一步迭代的法向位移
                delta_inner=delta_inner+delta_interval_inner;
                %初始法向位移迭代变化值减半
                delta_interval_inner=delta_interval_inner/10;
                kk=0;  %法向位移迭代变化值记数
            end
            %判断法各位移迭代变化值小于给定误差
           % if delta_interval_inner<10^-6
           %     break;
           %  end
        end
    end
    %记录滚动体与内圈切片接触应力向量（Mpa）,stress_inner
    stress_inner=zeros(number_slice,1);
    stress_inner(a_i_inner_first:a_i_inner_last,1)=contact_stress_inner;
end
%%%程序结束

%%%给定法向位移计算滚动体接触法向力、应力向量、接触半宽
function [contact_force,contact_stress,a_i,a_i_first,a_i_last,mark]=linear_contact_inter(R_total,hj,yyj,z_total,number_slice,E_syn,delta)
%%%参数输出
%滚动体接触法向力（N），contact_force
%切片接触应力向量（MPa），contact_stress
%切片实际接触半宽向量(mm)，a_i
%实际接触区最左边切片的序号，a_i_first
%实际接触区最右切片的序号，a_i_last
%初始状态滚动体切片接触状态，mark_initial，0表示有切片接触，1表示无切片接触
%%%参数输入
%滚子轴承滚动体的综合半径向量（mm），R_total
%滚动体切片半宽（mm），hj
%滚子轴承沿滚动体位置向量（中点为零点）（mm），yyj
%按切片个数计算滚动体的综合倾斜曲线（mm），z_total
%切片个数，number_slice
%滚子轴承滚动体的接触综合弹性模量（Mpa）,E_syn
%滚子轴承给定的法向位移量，delta
%%%程序开始
%%%定义迭代过程中滚动体切片接触状态，mark，0表示有切片接触，1表示无切片接触
global a_cnt;
global outloop_cnt;
global a_ccnt;
mark=0;
%%%定义迭代收敛标志，mark_convergence，0表示不收敛，1表示收敛
mark_convergence=0;
%%%计算给定法向位移时初始状态的切片受载变形向量及接触半宽向量
S_i=(delta-z_total).*(delta-z_total>0);   %切片受载变形向量（mm）
a_i=(2*R_total.*S_i).^0.5;   %切片接触半宽向量（mm）
outloop_cnt=outloop_cnt+1;
%%%给定法向位移时计算滚动体整体法向力及切片接触应力向量
a_ccnt=0;
if sum(a_i>0)==0  %给定法向位移时初始状态滚动体无切片接触
    mark=1;  %滚动体无切片接触
    contact_force=0;  %滚动体整体法向力（N），contact_force
    contact_stress=zeros(number_slice,1);  %滚动体接触应力向量（Mpa），contact_stress
    a_i=zeros(number_slice,1);  %滚动体接触半宽向量（mm），a_i
    a_i_first=1;
    a_i_last=number_slice;
else  %给定法向位移时初始状态滚动体有切片接触的状态
    %定义老状态的接触切片的接触半宽向量（mm），a_i_old
    a_i_old=a_i;
    %定义接触半宽迭代收敛的容差，tolerance_a_i
    tolerance_a_i=0.05;
    %定义复化辛普森积分点个数，n_sim
    n_sim=5;
    %定义给定法向位移时切片接触半宽迭代差值比例向量，a_i_dif
    a_i_dif=ones(number_slice,1);
    %定义给定法向位移时切片接触应力向量，contact_stress
    contact_stress=zeros(number_slice,1);
    %定义给定法向位移时切片接触状态向量，judge_contact_stress
    judge_contact_stress=(contact_stress<=0);    
    while (max(abs(a_i_dif))>tolerance_a_i)
        %%%计算给定法向位移时各切片接触状态，确定滚动体的接触边界
        while (sum(judge_contact_stress)>0)  %存在非接触的切片
            %记录滚动体接触切片位置向量，a_i_pos
            a_i_pos=find(a_i>0);
            if isempty(a_i_pos)==1  %所有切片均不接触
                mark=1;  %迭代过程中滚动体切片接触状态，mark，0表示有切片接触，1表示无切片接触
                break;
            end
            %计算滚动体接触切片最左序号，a_i_first；最右序号，a_i_last
            a_i_size=size(a_i_pos);
            a_i_length=a_i_size(1,1);
            a_i_first=a_i_pos(1,1);
            a_i_last=a_i_first+a_i_length-1;
            %记录接触切片受载变形向量（mm），S_i_D
            S_i_D=S_i(a_i_first:a_i_last,1);
            %%%计算接触切片的接触柔度系数矩阵，D
            [D]=contact_flex_matrix(n_sim,yyj,hj,a_i_first,a_i_last,a_i);
            %计算接触切片的接触应力向量（Mpa），contact_stress
            contact_stress=pi*E_syn*D^(-1)*S_i_D;
            %计算接触切片的接触综合半径向量（mm），R_total_calulate
            R_total_calulate=R_total(a_i_first:a_i_last,1);
            %计算新状态接触切片的接触半宽向量（mm），a_i_new
            a_i_calulate=2*R_total_calulate.*contact_stress/E_syn;
            a_i_new=zeros(number_slice,1);
            a_i_new(a_i_first:a_i_last,1)=a_i_calulate;
            %记录老状态的接触切片的接触半宽向量（mm），a_i_old
            a_i_old=a_i;
            %记录新状态的切片的接触状态向量，judge_contact_stress
            judge_contact_stress=(contact_stress<=0);
            %计算切片接触半宽迭代差值比例向量，a_i_dif
            for i=1:number_slice
                if a_i_new(i,1)~=0
                    a_i_dif(i,1)=(a_i_new(i,1)-a_i_old(i,1))/a_i_new(i,1);
                else
                    a_i_dif(i,1)=0;
                end
            end
            %判断是否迭代收敛
            if max(abs(a_i_dif))<=tolerance_a_i  %收敛
                a_i=a_i_new;
                mark_convergence=1;  %迭代收敛标志，mark_convergence，0表示不收敛，1表示收敛
                break;                
            else  %不收敛
                %计算下一迭代的接触切片的接触半宽向量（mm），a_i
                a_i=a_i_new;
            end
        end
        %%%在给定的法向位移，并在确定的接触边界上计算滚动体整体法向力及各切片的接触应力        
        if mark==1  %迭代过程中滚动体切片接触状态，mark，0表示有切片接触，1表示无切片接触
            contact_stress=zeros(number_slice,1);
            a_i=zeros(number_slice,1);
            break;
        end
        if mark_convergence==1;  %迭代收敛标志，mark_convergence，0表示不收敛，1表示收敛
            break;
        end
        %计算下一迭代的接触半宽向量（mm），a_i
        a_i=0.5*(a_i+a_i_old);
        a_cnt=a_cnt+1;
        %%%计算接触柔度系数矩阵，D
        [D]=contact_flex_matrix(n_sim,yyj,hj,a_i_first,a_i_last,a_i);
        %计算各切片接触应力向量（Mpa）
        contact_stress=pi*E_syn*D^(-1)*S_i_D;
        %记录接触状态的综合曲率半径向量(mm)
        R_total_calulate=R_total(a_i_first:a_i_last,1);
        %计算接触状态的接触半宽向量(mm)，a_i_calulate
        a_i_calulate=2*R_total_calulate.*contact_stress/E_syn;
        %记录新状态的接触半宽向量（mm），a_i_new
        a_i_new=zeros(number_slice,1);
        a_i_new(a_i_first:a_i_last,1)=a_i_calulate;
        %记录老状态的接触半宽向量（mm），a_i
        a_i_old=a_i;
        a_i=a_i_new;
        %计算新状态的切片接触状态向量，judge_contact_stress
        judge_contact_stress=(contact_stress<=0);
        %计算切片接触半宽迭代差值比例向量，a_i_dif
        for i=1:number_slice
            if a_i(i,1)~=0
                a_i_dif(i,1)=(a_i(i,1)-a_i_old(i,1))/a_i(i,1);
            else
                a_i_dif(i,1)=0;
            end
        end
        a_ccnt=a_ccnt+1;
    end
    if mark==1  %迭代过程中滚动体切片接触状态，mark，0表示有切片接触，1表示无切片接触
        %计算滚动体整体法向力
        contact_force=0;
    else
        %计算滚动体整体法向力
        contact_force=pi*hj*a_i_calulate'*contact_stress;
    end
    a_ccnt
end
%%%程序结束

%%%计算接触柔度系数矩阵，D
function [D]=contact_flex_matrix(n_sim,yyj,hj,a_i_first,a_i_last,a_i)
%%%参数输出
%滚动体的接触柔度矩阵，D
%%%参数输入
%复化辛普森积分参数，n_sim
%滚子轴承沿滚动体位置向量（中点为零点）（mm），yyj
%滚动体切片半宽（mm），hj
%滚动体实际接触最左切片序号，a_i_first
%滚动体实际接触最右切片序号，a_i_last
%滚动体切片接触半宽向量（mm），a_i
%%%程序开始
global D_cnt
global time_cnt;
tic;
t3=clock;
p=a_i_last-a_i_first+1;
D=zeros(p,p);
m=a_i_first-1;
mm=0;
for j=a_i_first:a_i_last
    m=m+1;
    mm=mm+1;
    yj=yyj(m,1);
    con_semi_major=a_i(m,1);
    n=a_i_first-1;
    nn=0;
    for i=a_i_first:a_i_last
        n=n+1;
        nn=nn+1;
        yi=yyj(n,1);
        if con_semi_major==0
            D_ij=0;
        else
            [D_ij]=echelon_d(n_sim,yi,yj,con_semi_major,hj);
        end
        D(nn,mm)=D_ij
    end
end
D_cnt=D_cnt+1;
time_cnt=time_cnt+etime(clock,t3);
disp(['计算接触柔度系数矩阵D,时间：',num2str(etime(clock,t3))]);
%D
%%%

%%%计算函数的复化梯形数积分
function [D_ij]=echelon_d(n_sim,yi,yj,aj,hj)  %复化梯形数值积分
%global dij_cnt;
%%%参数输出
%滚动体的接触柔度矩阵，D_ij
%%%参数输入
%复化辛普森积分参数，n_sim
%第i个切片中点位置（mm），yi
%第j个切片中点位置（mm），yj
%第j个切片的接触半宽（mm），aj
%切片半宽（mm），hj
%%%程序开始
tic;
t5=clock;
f=@(x)(1-(x/aj)^2)^(1/2)*log((abs(yi-yj)+hj+(x^2+(abs(yi-yj)+hj)^2)^(1/2))/(abs(yi-yj)-hj+(x^2+(abs(yi-yj)-hj)^2)^(1/2)));
n_sim=2*ceil(abs(n_sim))+1;
h_sim=2*aj/n_sim;
S=feval(f,-aj);
for i=1:n_sim-1
    xi=-aj+h_sim*i;
    S=S+2*feval(f,xi);
end
S=S+feval(f,aj);
D_ij=0.5*h_sim*S;
%disp(['计算函数的复化梯形数积分Dij,时间：',num2str(etime(clock,t5))]);
%%%程序结束