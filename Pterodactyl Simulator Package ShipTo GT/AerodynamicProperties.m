function AerodynamicProperties
% Function File to Designate Aerodynamic Properties for the NACA 0012
% Airfoil used on the Pterodactyl

% Change Log
%{
    10/2012 - Function Written by Trevor Bennett
    5/2013 - Control Surface Average Values Added by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}


 tic
%% Air Properties %%
% Dynamic Viscosity of Air
temp = 70;              % Temperature in degrees F
fit = 0.0051*temp + 3.4292;
mu = fit*1E-7;          % [ lbsf/ft^2]

%% Input Aerodynamic Properties %%
    % The Pterodactyle Currently Utilizes a NACA 0012 Airfoil
    %{
        ORIGINAL SOURCE: Sheldahl, R. E. and Klimas, P. C., Aerodynamic 
        Characteristics of Seven Airfoil Sections Through 
        180 Degrees Angle of Attack for Use in Aerodynamic 
        Analysis of Vertical Axis Wind Turbines, SAND80-2114, 
        March 1981, Sandia National Laboratories, Albuquerque, 
        New Mexico
    
        Data Pulled from (respectively)
            http://www.cyberiad.net/library/airfoils/foildata/n0012cl.htm
            http://www.cyberiad.net/library/airfoils/foildata/n0012cd.htm
            http://www.cyberiad.net/library/airfoils/foildata/n0012cm.htm
    %}

%%% Lift Coefficient Data %%%
AlphaClin = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,...
    23,24,25,26,27,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,...
    115,120,125,130,135,140,145,150,155,160,165,170,175,180,185,190,195,...
    200,205,210,215,220,225,230,235,240,245,250,255,260,265,270,275,280,...
    285,290,295,300,305,310,315,320,325,330,333,334,335,336,337,338,339,...
    340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,...
    357,358,359,360]';

ReClin = [160000,360000,700000,1000000,2000000,5000000];

ClAlphain = ...
    [0         0         0         0         0         0
    0.1100    0.1100    0.1100    0.1100    0.1100    0.1100
    0.2200    0.2200    0.2200    0.2200    0.2200    0.2200
    0.3300    0.3300    0.3300    0.3300    0.3300    0.3300
    0.4400    0.4400    0.4400    0.4400    0.4400    0.4400
    0.5500    0.5500    0.5500    0.5500    0.5500    0.5500
    0.6600    0.6600    0.6600    0.6600    0.6600    0.6600
    0.7460    0.7700    0.7700    0.7700    0.7700    0.7700
    0.8247    0.8542    0.8800    0.8800    0.8800    0.8800
    0.8527    0.9352    0.9598    0.9661    0.9900    0.9900
    0.1325    0.9811    1.0343    1.0512    1.0727    1.1000
    0.1095    0.9132    1.0749    1.1097    1.1539    1.1842
    0.1533    0.4832    1.0390    1.1212    1.2072    1.2673
    0.2030    0.2759    0.8737    1.0487    1.2169    1.3242
    0.2546    0.2893    0.6284    0.8846    1.1614    1.3423
    0.3082    0.3306    0.4907    0.7108    1.0478    1.3093
    0.3620    0.3792    0.4696    0.6060    0.9221    1.2195
    0.4200    0.4455    0.5195    0.5906    0.7826    1.0365
    0.4768    0.5047    0.5584    0.6030    0.7163    0.9054
    0.5322    0.5591    0.6032    0.6334    0.7091    0.8412
    0.5870    0.6120    0.6474    0.6716    0.7269    0.8233
    0.6414    0.6643    0.6949    0.7162    0.7595    0.8327
    0.6956    0.7179    0.7446    0.7613    0.7981    0.8563
    0.7497    0.7715    0.7948    0.8097    0.8429    0.8903
    0.8043    0.8246    0.8462    0.8589    0.8882    0.9295
    0.8572    0.8780    0.8984    0.9093    0.9352    0.9718
    0.9109    0.9313    0.9506    0.9618    0.9842    1.0193
    0.9230    0.9412    0.9583    0.9683    0.9882    1.0680
    0.9593    0.9709    0.9814    0.9878    1.0020    0.9150
    1.0200    1.0200    1.0200    1.0200    1.0200    1.0200
    1.0750    1.0750    1.0750    1.0750    1.0750    1.0750
    1.0850    1.0850    1.0850    1.0850    1.0850    1.0850
    1.0400    1.0400    1.0400    1.0400    1.0400    1.0400
    0.9650    0.9650    0.9650    0.9650    0.9650    0.9650
    0.8750    0.8750    0.8750    0.8750    0.8750    0.8750
    0.7650    0.7650    0.7650    0.7650    0.7650    0.7650
    0.6500    0.6500    0.6500    0.6500    0.6500    0.6500
    0.5150    0.5150    0.5150    0.5150    0.5150    0.5150
    0.3700    0.3700    0.3700    0.3700    0.3700    0.3700
    0.2200    0.2200    0.2200    0.2200    0.2200    0.2200
    0.0700    0.0700    0.0700    0.0700    0.0700    0.0700
   -0.0700   -0.0700   -0.0700   -0.0700   -0.0700   -0.0700
   -0.2200   -0.2200   -0.2200   -0.2200   -0.2200   -0.2200
   -0.3700   -0.3700   -0.3700   -0.3700   -0.3700   -0.3700
   -0.5100   -0.5100   -0.5100   -0.5100   -0.5100   -0.5100
   -0.6250   -0.6250   -0.6250   -0.6250   -0.6250   -0.6250
   -0.7350   -0.7350   -0.7350   -0.7350   -0.7350   -0.7350
   -0.8400   -0.8400   -0.8400   -0.8400   -0.8400   -0.8400
   -0.9100   -0.9100   -0.9100   -0.9100   -0.9100   -0.9100
   -0.9450   -0.9450   -0.9450   -0.9450   -0.9450   -0.9450
   -0.9450   -0.9450   -0.9450   -0.9450   -0.9450   -0.9450
   -0.9100   -0.9100   -0.9100   -0.9100   -0.9100   -0.9100
   -0.8500   -0.8500   -0.8500   -0.8500   -0.8500   -0.8500
   -0.7400   -0.7400   -0.7400   -0.7400   -0.7400   -0.7400
   -0.6600   -0.6600   -0.6600   -0.6600   -0.6600   -0.6600
   -0.6750   -0.6750   -0.6750   -0.6750   -0.6750   -0.6750
   -0.8500   -0.8500   -0.8500   -0.8500   -0.8500   -0.8500
   -0.6900   -0.6900   -0.6900   -0.6900   -0.6900   -0.6900
         0         0         0         0         0         0
    0.6900    0.6900    0.6900    0.6900    0.6900    0.6900
    0.8500    0.8500    0.8500    0.8500    0.8500    0.8500
    0.6750    0.6750    0.6750    0.6750    0.6750    0.6750
    0.6600    0.6600    0.6600    0.6600    0.6600    0.6600
    0.7400    0.7400    0.7400    0.7400    0.7400    0.7400
    0.8500    0.8500    0.8500    0.8500    0.8500    0.8500
    0.9100    0.9100    0.9100    0.9100    0.9100    0.9100
    0.9450    0.9450    0.9450    0.9450    0.9450    0.9450
    0.9450    0.9450    0.9450    0.9450    0.9450    0.9450
    0.9100    0.9100    0.9100    0.9100    0.9100    0.9100
    0.8400    0.8400    0.8400    0.8400    0.8400    0.8400
    0.7350    0.7350    0.7350    0.7350    0.7350    0.7350
    0.6250    0.6250    0.6250    0.6250    0.6250    0.6250
    0.5100    0.5100    0.5100    0.5100    0.5100    0.5100
    0.3700    0.3700    0.3700    0.3700    0.3700    0.3700
    0.2200    0.2200    0.2200    0.2200    0.2200    0.2200
    0.0700    0.0700    0.0700    0.0700    0.0700    0.0700
   -0.0700   -0.0700   -0.0700   -0.0700   -0.0700   -0.0700
   -0.2200   -0.2200   -0.2200   -0.2200   -0.2200   -0.2200
   -0.3700   -0.3700   -0.3700   -0.3700   -0.3700   -0.3700
   -0.5150   -0.5150   -0.5150   -0.5150   -0.5150   -0.5150
   -0.6500   -0.6500   -0.6500   -0.6500   -0.6500   -0.6500
   -0.7650   -0.7650   -0.7650   -0.7650   -0.7650   -0.7650
   -0.8750   -0.8750   -0.8750   -0.8750   -0.8750   -0.8750
   -0.9650   -0.9650   -0.9650   -0.9650   -0.9650   -0.9650
   -1.0400   -1.0400   -1.0400   -1.0400   -1.0400   -1.0400
   -1.0850   -1.0850   -1.0850   -1.0850   -1.0850   -1.0850
   -1.0750   -1.0750   -1.0750   -1.0750   -1.0750   -1.0750
   -1.0200   -1.0200   -1.0200   -1.0200   -1.0200   -1.0200
   -0.9593   -0.9709   -0.9814   -0.9878   -1.0002   -0.9150
   -0.9230   -0.9412   -0.9583   -0.9683   -0.9882   -1.0680
   -0.9109   -0.9313   -0.9506   -0.9618   -0.9842   -1.0193
   -0.8572   -0.8780   -0.8984   -0.9093   -0.9352   -0.9718
   -0.8043   -0.8246   -0.8462   -0.8589   -0.8882   -0.9295
   -0.7497   -0.7715   -0.7948   -0.8097   -0.8429   -0.8903
   -0.6956   -0.7179   -0.7446   -0.7613   -0.7981   -0.8563
   -0.6414   -0.6643   -0.6949   -0.7162   -0.7595   -0.8327
   -0.5870   -0.6120   -0.6474   -0.6716   -0.7269   -0.8233
   -0.5322   -0.5591   -0.6032   -0.6334   -0.7091   -0.8412
   -0.4768   -0.5047   -0.5584   -0.6030   -0.7163   -0.9054
   -0.4200   -0.4455   -0.5195   -0.5906   -0.7826   -1.0365
   -0.3620   -0.3792   -0.4696   -0.6060   -0.9221   -1.2195
   -0.3082   -0.3306   -0.4907   -0.7108   -1.0478   -1.3093
   -0.2546   -0.2893   -0.6284   -0.8846   -1.1614   -1.3423
   -0.2030   -0.2759   -0.8737   -1.0487   -1.2169   -1.3242
   -0.1533   -0.4832   -1.0390   -1.1212   -1.2072   -1.2673
   -0.1095   -0.9132   -1.0749   -1.1097   -1.1539   -1.1842
   -0.1325   -0.9811   -1.0343   -1.0512   -1.0727   -1.1000
   -0.8527   -0.9352   -0.9598   -0.9661   -0.9900   -0.9900
   -0.8247   -0.8542   -0.8800   -0.8800   -0.8800   -0.8800
   -0.7460   -0.7700   -0.7700   -0.7700   -0.7700   -0.7700
   -0.6600   -0.6600   -0.6600   -0.6600   -0.6600   -0.6600
   -0.5500   -0.5500   -0.5500   -0.5500   -0.5500   -0.5500
   -0.4400   -0.4400   -0.4400   -0.4400   -0.4400   -0.4400
   -0.3300   -0.3300   -0.3300   -0.3300   -0.3300   -0.3300
   -0.2200   -0.2200   -0.2200   -0.2200   -0.2200   -0.2200
   -0.1100   -0.1100   -0.1100   -0.1100   -0.1100   -0.1100
   0         0         0         0         0         0];


%%% Drag Coefficient Data %%%
AlphaCdin = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,...
    23,24,25,26,27,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,...
    115,120,125,130,135,140,145,150,155,160,165,170,175,180,185,190,195,...
    200,205,210,215,220,225,230,235,240,245,250,255,260,265,270,275,280,...
    285,290,295,300,305,310,315,320,325,330,333,334,335,336,337,338,339,...
    340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,...
    357,358,359,360]';

ReCdin = [160000,360000,700000,1000000,2000000,5000000];

CdAlphain = ...
   [0.0103    0.0079    0.0067    0.0065    0.0064    0.0064
    0.0104    0.0080    0.0068    0.0066    0.0064    0.0064
    0.0108    0.0084    0.0070    0.0068    0.0066    0.0066
    0.0114    0.0089    0.0075    0.0071    0.0069    0.0068
    0.0124    0.0098    0.0083    0.0078    0.0073    0.0072
    0.0140    0.0113    0.0097    0.0091    0.0081    0.0076
    0.0152    0.0125    0.0108    0.0101    0.0090    0.0081
    0.0170    0.0135    0.0118    0.0110    0.0097    0.0086
    0.0185    0.0153    0.0128    0.0119    0.0105    0.0092
    0.0203    0.0167    0.0144    0.0134    0.0113    0.0098
    0.0188    0.0184    0.0159    0.0147    0.0128    0.0106
    0.0760    0.0204    0.0175    0.0162    0.0140    0.0118
    0.1340    0.0217    0.0195    0.0180    0.0155    0.0130
    0.1520    0.0222    0.0216    0.0200    0.0172    0.0143
    0.1710    0.1060    0.0236    0.0222    0.0191    0.0159
    0.1900    0.1900    0.1170    0.0245    0.0213    0.0177
    0.2100    0.2100    0.2100    0.1280    0.0237    0.0198
    0.2310    0.2310    0.2300    0.2310    0.1380    0.0229
    0.2520    0.2520    0.2520    0.2520    0.2520    0.1480
    0.2740    0.2740    0.2740    0.2740    0.2740    0.2740
    0.2970    0.2970    0.2970    0.2970    0.2970    0.2970
    0.3200    0.3200    0.3200    0.3200    0.3200    0.3200
    0.3440    0.3440    0.3440    0.3440    0.3440    0.3440
    0.3690    0.3690    0.3690    0.3690    0.3690    0.3690
    0.3940    0.3940    0.3940    0.3940    0.3940    0.3940
    0.4200    0.4200    0.4200    0.4200    0.4200    0.4200
    0.4460    0.4460    0.4460    0.4460    0.4460    0.4460
    0.4730    0.4730    0.4730    0.4730    0.4730    0.4730
    0.5700    0.5700    0.5700    0.5700    0.5700    0.5700
    0.7450    0.7450    0.7450    0.7450    0.7450    0.7450
    0.9200    0.9200    0.9200    0.9200    0.9200    0.9200
    1.0750    1.0750    1.0750    1.0750    1.0750    1.0750
    1.2150    1.2150    1.2150    1.2150    1.2150    1.2150
    1.3450    1.3450    1.3450    1.3450    1.3450    1.3450
    1.4700    1.4700    1.4700    1.4700    1.4700    1.4700
    1.5750    1.5750    1.5750    1.5750    1.5750    1.5750
    1.6650    1.6650    1.6650    1.6650    1.6650    1.6650
    1.7350    1.7350    1.7350    1.7350    1.7350    1.7350
    1.7800    1.7800    1.7800    1.7800    1.7800    1.7800
    1.8000    1.8000    1.8000    1.8000    1.8000    1.8000
    1.8000    1.8000    1.8000    1.8000    1.8000    1.8000
    1.7800    1.7800    1.7800    1.7800    1.7800    1.7800
    1.7500    1.7500    1.7500    1.7500    1.7500    1.7500
    1.7000    1.7000    1.7000    1.7000    1.7000    1.7000
    1.6350    1.6350    1.6350    1.6350    1.6350    1.6350
    1.5550    1.5550    1.5550    1.5550    1.5550    1.5550
    1.4650    1.4650    1.4650    1.4650    1.4650    1.4650
    1.3500    1.3500    1.3500    1.3500    1.3500    1.3500
    1.2250    1.2250    1.2250    1.2250    1.2250    1.2250
    1.0850    1.0850    1.0850    1.0850    1.0850    1.0850
    0.9250    0.9250    0.9250    0.9250    0.9250    0.9250
    0.7550    0.7550    0.7550    0.7550    0.7550    0.7550
    0.5750    0.5750    0.5750    0.5750    0.5750    0.5750
    0.4200    0.4200    0.4200    0.4200    0.4200    0.4200
    0.3200    0.3200    0.3200    0.3200    0.3200    0.3200
    0.2300    0.2300    0.2300    0.2300    0.2300    0.2300
    0.1400    0.1400    0.1400    0.1400    0.1400    0.1400
    0.0550    0.0550    0.0550    0.0550    0.0550    0.0550
    0.0250    0.0250    0.0250    0.0250    0.0250    0.0250
    0.0550    0.0550    0.0550    0.0550    0.0550    0.0550
    0.1400    0.1400    0.1400    0.1400    0.1400    0.1400
    0.2300    0.2300    0.2300    0.2300    0.2300    0.2300
    0.3200    0.3200    0.3200    0.3200    0.3200    0.3200
    0.4200    0.4200    0.4200    0.4200    0.4200    0.4200
    0.5750    0.5750    0.5750    0.5750    0.5750    0.5750
    0.7550    0.7550    0.7550    0.7550    0.7550    0.7550
    0.9250    0.9250    0.9250    0.9250    0.9250    0.9250
    1.0850    1.0850    1.0850    1.0850    1.0850    1.0850
    1.2250    1.2250    1.2250    1.2250    1.2250    1.2250
    1.3500    1.3500    1.3500    1.3500    1.3500    1.3500
    1.4650    1.4650    1.4650    1.4650    1.4650    1.4650
    1.5550    1.5550    1.5550    1.5550    1.5550    1.5550
    1.6350    1.6350    1.6350    1.6350    1.6350    1.6350
    1.7000    1.7000    1.7000    1.7000    1.7000    1.7000
    1.7500    1.7500    1.7500    1.7500    1.7500    1.7500
    1.7800    1.7800    1.7800    1.7800    1.7800    1.7800
    1.8000    1.8000    1.8000    1.8000    1.8000    1.8000
    1.8000    1.8000    1.8000    1.8000    1.8000    1.8000
    1.7800    1.7800    1.7800    1.7800    1.7800    1.7800
    1.7350    1.7350    1.7350    1.7350    1.7350    1.7350
    1.6650    1.6650    1.6650    1.6650    1.6650    1.6650
    1.5750    1.5750    1.5750    1.5750    1.5750    1.5750
    1.4700    1.4700    1.4700    1.4700    1.4700    1.4700
    1.3450    1.3450    1.3450    1.3450    1.3450    1.3450
    1.2150    1.2150    1.2150    1.2150    1.2150    1.2150
    1.0750    1.0750    1.0750    1.0750    1.0750    1.0750
    0.9200    0.9200    0.9200    0.9200    0.9200    0.9200
    0.7450    0.7450    0.7450    0.7450    0.7450    0.7450
    0.5700    0.5700    0.5700    0.5700    0.5700    0.5700
    0.4730    0.4730    0.4730    0.4730    0.4730    0.4730
    0.4460    0.4460    0.4460    0.4460    0.4460    0.4460
    0.4200    0.4200    0.4200    0.4200    0.4200    0.4200
    0.3940    0.3940    0.3940    0.3940    0.3940    0.3940
    0.3690    0.3690    0.3690    0.3690    0.3690    0.3690
    0.3440    0.3440    0.3440    0.3440    0.3440    0.3440
    0.3200    0.3200    0.3200    0.3200    0.3200    0.3200
    0.2970    0.2970    0.2970    0.2970    0.2970    0.2970
    0.2740    0.2740    0.2740    0.2740    0.2740    0.2740
    0.2520    0.2520    0.2520    0.2520    0.2520    0.1480
    0.2310    0.2310    0.2300    0.2310    0.1380    0.0229
    0.2100    0.2100    0.2100    0.1280    0.0237    0.0198
    0.1900    0.1900    0.1170    0.0245    0.0213    0.0177
    0.1710    0.1060    0.0236    0.0222    0.0191    0.0159
    0.1520    0.0222    0.0216    0.0200    0.0172    0.0143
    0.1340    0.0217    0.0195    0.0180    0.0155    0.0130
    0.0760    0.0204    0.0175    0.0162    0.0140    0.0118
    0.0188    0.0184    0.0159    0.0147    0.0128    0.0106
    0.0203    0.0167    0.0144    0.0134    0.0113    0.0098
    0.0185    0.0153    0.0128    0.0119    0.0105    0.0092
    0.0170    0.0135    0.0118    0.0110    0.0097    0.0086
    0.0152    0.0125    0.0108    0.0101    0.0090    0.0081
    0.0140    0.0113    0.0097    0.0091    0.0081    0.0076
    0.0124    0.0098    0.0083    0.0078    0.0073    0.0072
    0.0114    0.0089    0.0075    0.0071    0.0069    0.0068
    0.0108    0.0084    0.0070    0.0068    0.0066    0.0066
    0.0104    0.0080    0.0068    0.0066    0.0064    0.0064
    0.0103    0.0079    0.0067    0.0065    0.0064    0.0064];


%%% Moment Coefficient Data %%%
% Moment About the Quater Chord
AlphaCmin = [0,2,4,6,8,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,34,38,...
    40,42.5,45,50,55,60,65,70,75,80,85,86,90,91,95,100,105,110,115,120,...
    125,130,131,135,136,140,145,150,155,160,165,170,175,180,185,190,195,...
    200,205,210,215,220,224,225,229,230,235,240,245,250,255,260,265,269,...
    270,274,275,280,285,290,295,300,305,310,315,317.5,320,322,326,330,332,...
    334,336,338,340,342,343,344,345,346,347,348,349,350,352,354,356,358,360]';

ReCmin = [360000,500000,700000,860000,1360000,1760000];

CmAlphain = ...
   [ 0         0         0         0         0         0
   -0.0100   -0.0050    0.0070    0.0040    0.0060         0
   -0.0220   -0.0100    0.0080    0.0070    0.0100    0.0030
   -0.0120    0.0060    0.0030   -0.0040    0.0030    0.0040
    0.0210    0.0220    0.0160    0.0050    0.0050         0
    0.0260    0.0370    0.0300    0.0090    0.0130    0.0090
    0.0120    0.0400    0.0340    0.0170    0.0160    0.0130
   -0.0500   -0.0210   -0.0220    0.0190    0.0180    0.0170
   -0.0590   -0.0500   -0.0500    0.0240    0.0210    0.0200
   -0.0790   -0.0610   -0.0600   -0.0160    0.0010    0.0210
   -0.0810   -0.0680   -0.0610   -0.0230   -0.0330   -0.0500
   -0.0880   -0.0620   -0.0640   -0.0620   -0.0410   -0.0530
   -0.0830   -0.0700   -0.0700   -0.0820   -0.0470   -0.0620
   -0.0750   -0.0810   -0.0760   -0.0800   -0.0850   -0.0540
   -0.0810   -0.0630   -0.0670   -0.0900   -0.0800   -0.0990
   -0.0680   -0.0650   -0.0760   -0.0820   -0.0780   -0.0940
   -0.0740   -0.1080   -0.0780   -0.1000   -0.0910   -0.0960
   -0.0850   -0.0950   -0.0900   -0.1020   -0.1100   -0.1190
   -0.0950   -0.0970   -0.0980   -0.1100   -0.1350   -0.1480
   -0.1030   -0.1000   -0.1100   -0.1400   -0.1250   -0.1460
   -0.0950   -0.1500   -0.1700   -0.1700   -0.1700   -0.1700
   -0.1200   -0.2200   -0.1800   -0.1800   -0.1800   -0.1800
   -0.2050   -0.1950   -0.2250   -0.2250   -0.2250   -0.2250
   -0.2490   -0.2100   -0.1400   -0.1400   -0.1400   -0.1400
   -0.2250   -0.1600   -0.1800   -0.1800   -0.1800   -0.1800
   -0.1550   -0.1250   -0.2600   -0.2600   -0.2600   -0.2600
   -0.3600   -0.2000   -0.3200   -0.3200   -0.3200   -0.3200
   -0.2400   -0.2600   -0.2800   -0.2800   -0.2800   -0.2800
   -0.2800   -0.2300   -0.2950   -0.2950   -0.2950   -0.2950
   -0.3300   -0.3800   -0.2850   -0.2850   -0.2850   -0.2850
   -0.2700   -0.2600   -0.3600   -0.3600   -0.3600   -0.3600
   -0.4100   -0.3450   -0.1900   -0.1900   -0.1900   -0.1900
   -0.2900   -0.3350   -0.2900   -0.2900   -0.2900   -0.2900
   -0.3300   -0.3650   -0.3850   -0.3850   -0.3850   -0.3850
   -0.3550   -0.1750   -0.2800   -0.2800   -0.2800   -0.2800
   -0.4100   -0.3200   -0.4600   -0.4600   -0.4600   -0.4600
   -0.2400   -0.4200   -0.3550   -0.3550   -0.3550   -0.3550
   -0.3600   -0.3900   -0.5250   -0.5250   -0.5250   -0.5250
   -0.6250   -0.4100   -0.2750   -0.2750   -0.2750   -0.2750
   -0.4900   -0.4200   -0.3400   -0.3400   -0.3400   -0.3400
   -0.3300   -0.4100   -0.3800   -0.3800   -0.3800   -0.3800
   -0.4200   -0.4850   -0.3200   -0.3200   -0.3200   -0.3200
   -0.3900   -0.5150   -0.4800   -0.4800   -0.4800   -0.4800
   -0.3900   -0.4600   -0.3600   -0.3600   -0.3600   -0.3600
   -0.4850   -0.4750   -0.3800   -0.3800   -0.3800   -0.3800
   -0.4050   -0.4350   -0.4050   -0.4050   -0.4050   -0.4050
   -0.5550   -0.4450   -0.4250   -0.4250   -0.4250   -0.4250
   -0.4550   -0.3800   -0.5150   -0.5150   -0.5150   -0.5150
   -0.4050   -0.3900   -0.4300   -0.4300   -0.4300   -0.4300
   -0.3850   -0.3750   -0.4050   -0.4050   -0.4050   -0.4050
   -0.3200   -0.3100   -0.3350   -0.3350   -0.3350   -0.3350
   -0.2950   -0.2850   -0.3000   -0.3000   -0.3000   -0.3000
   -0.2900   -0.2700   -0.2800   -0.2800   -0.2800   -0.2800
   -0.3750   -0.3500   -0.3800   -0.3800   -0.3800   -0.3800
   -0.2900   -0.2850   -0.3100   -0.3100   -0.3100   -0.3100
    0.0250    0.0450   -0.0200   -0.0200   -0.0200   -0.0200
    0.2900    0.2850    0.3100    0.3100    0.3100    0.3100
    0.3750    0.3500    0.3800    0.3800    0.3800    0.3800
    0.2900    0.2700    0.2800    0.2800    0.2800    0.2800
    0.2950    0.2850    0.3000    0.3000    0.3000    0.3000
    0.3200    0.3100    0.3350    0.3350    0.3350    0.3350
    0.3850    0.3750    0.4050    0.4050    0.4050    0.4050
    0.4050    0.3900    0.4300    0.4300    0.4300    0.4300
    0.4550    0.3800    0.5150    0.5150    0.5150    0.5150
    0.5550    0.4450    0.4250    0.4250    0.4250    0.4250
    0.4050    0.4350    0.4050    0.4050    0.4050    0.4050
    0.4850    0.4750    0.3800    0.3800    0.3800    0.3800
    0.3900    0.4600    0.3600    0.3600    0.3600    0.3600
    0.3900    0.5150    0.4800    0.4800    0.4800    0.4800
    0.4200    0.4850    0.3200    0.3200    0.3200    0.3200
    0.3300    0.4100    0.3800    0.3800    0.3800    0.3800
    0.4900    0.4200    0.3400    0.3400    0.3400    0.3400
    0.6250    0.4100    0.2750    0.2750    0.2750    0.2750
    0.3600    0.3900    0.5250    0.5250    0.5250    0.5250
    0.2400    0.4200    0.3550    0.3550    0.3550    0.3550
    0.4100    0.3200    0.4600    0.4600    0.4600    0.4600
    0.3550    0.1750    0.2800    0.2800    0.2800    0.2800
    0.3300    0.3650    0.3850    0.3850    0.3850    0.3850
    0.2900    0.3350    0.2900    0.2900    0.2900    0.2900
    0.4100    0.3450    0.1900    0.1900    0.1900    0.1900
    0.2700    0.2600    0.3600    0.3600    0.3600    0.3600
    0.3300    0.3800    0.2850    0.2850    0.2850    0.2850
    0.2800    0.2300    0.2950    0.2950    0.2950    0.2950
    0.2400    0.2600    0.2800    0.2800    0.2800    0.2800
    0.3600    0.2000    0.3200    0.3200    0.3200    0.3200
    0.1550    0.1250    0.2600    0.2600    0.2600    0.2600
    0.2250    0.1600    0.1800    0.1800    0.1800    0.1800
    0.2490    0.2100    0.1400    0.1400    0.1400    0.1400
    0.2050    0.1950    0.2250    0.2250    0.2250    0.2250
    0.1200    0.2200    0.1800    0.1800    0.1800    0.1800
    0.0950    0.1500    0.1700    0.1700    0.1700    0.1700
    0.1030    0.1000    0.1100    0.1400    0.1250    0.1460
    0.0950    0.0970    0.0980    0.1100    0.1350    0.1480
    0.0850    0.0950    0.0900    0.1020    0.1100    0.1190
    0.0790    0.0980    0.1150    0.1150    0.1150    0.1150
    0.0760    0.0970    0.1060    0.1060    0.1060    0.1060
    0.0620    0.0950    0.0990    0.1030    0.1080    0.1000
    0.0620    0.0900    0.0870    0.0960    0.0970    0.0980
    0.0690    0.0790    0.0820    0.0920    0.0990    0.1000
    0.0740    0.0720    0.0800    0.0950    0.0950    0.0970
    0.0720    0.0760    0.0850    0.0750    0.0880    0.0420
    0.0540    0.0710    0.0410    0.0130    0.0290   -0.0030
    0.0360    0.0400   -0.0310   -0.0180   -0.0150   -0.0200
   -0.0260   -0.0250   -0.0280   -0.0160   -0.0130   -0.0180
   -0.0260   -0.0240   -0.0230   -0.0130   -0.0090   -0.0140
   -0.0260   -0.0200   -0.0150   -0.0120   -0.0070   -0.0100
   -0.0130   -0.0050   -0.0050   -0.0080         0   -0.0030
   -0.0050         0         0         0   -0.0010   -0.0100
   -0.0050   -0.0020   -0.0120   -0.0090   -0.0060   -0.0110
   -0.0170   -0.0070   -0.0030   -0.0050   -0.0020   -0.0060
   0         0         0         0         0         0];

%% Data Rotation to Airfoil Frame %%
% The rotation of the cl and cd data rotates the lift and drag forces in
% the wind frame into the airfoil fixed frame.

% Allocate Memory
% Assuming the drag and lift measurements occured at same alpha increments
r = length(AlphaClin);            
c = length(ReClin);               % Corresponding number of data sets in Re
cfzAlpha = zeros(r,c);
cfxAlpha = zeros(r,c);   
for i = 1:r
    alpha = AlphaClin(i,1);
    Ca = cosd(alpha);
    Sa = sind(alpha);
    for j = 1:c
       % Negative because lift is in -z direction
       % Negative because drag is in -x direction
       cfzAlpha(i,j) = -(Ca*ClAlphain(i,j) + Sa*CdAlphain(i,j));     
       cfxAlpha(i,j) = -(-Sa*ClAlphain(i,j) + Ca*CdAlphain(i,j));
    end 
end

%% Re-Mesh the Data %%
% Transfer Data
ReCF = ReClin;
ReCm = ReCmin;

% Mesh Accuracy
Mesh = 0.1;
MeshMult = 1/Mesh;

% AlphaCF
Alpha = (0:Mesh:360)';
lalph = length(Alpha);

% Allocate Memory
CFxAlpha = zeros(lalph,c);
CFzAlpha = zeros(lalph,c);
CmAlpha = zeros(lalph,c);

% Populate Matricies
for i = 1:lalph
    alph = Alpha(i,1);
    for j = 1:c
        CFxAlpha(i,j) = interp2(ReCF,AlphaClin,cfxAlpha,ReCF(1,j),alph,'spline');
        CFzAlpha(i,j) = interp2(ReCF,AlphaClin,cfzAlpha,ReCF(1,j),alph,'spline');
        CmAlpha(i,j) = interp2(ReCF,AlphaCmin,CmAlphain,ReCF(1,j),alph,'spline');
    end   
end

%% Control Surface Deflection Coefficients %%
% Assuming Linear fits within -amax <= del_alpha <= amax. This assumption
% is only reasonable if the primary flow over the control surface is from
% the propeller wash and that the effective change in angle of attack is 
% small.
amax = 5;
lena = amax/Mesh;

% CFx
CFxFitData = zeros(2*lena*c+1,2);
for ind = 1:c
  top = CFxAlpha(1:lena,ind);
  ta = Alpha(1:lena,1);
  bottom = CFxAlpha(end-lena+1:end,ind);
  tb = Alpha(end-lena+1:end,1) - 360;
  CFxFitData(2*lena*(ind-1)+1:2*lena*ind,1) = [tb;ta];
  CFxFitData(2*lena*(ind-1)+1:2*lena*ind,2) = [bottom;top];
end
% Parabolic Fit - Best
CFxFit = polyfit(CFxFitData(:,1),CFxFitData(:,2),2);
CFxCoeff = CFxFit(end-1);
% Linear Fit - Best for state space control - not good values
% CFxLinFit = polyfit(CFxFitData(:,1),CFxFitData(:,2),1);
% CFxCoeff = CFxLinFit(end-1);

% CFz
CFzFitData = zeros(2*lena*c+1,2);
for ind = 1:c
  top = CFzAlpha(1:lena,ind);
  ta = Alpha(1:lena,1);
  bottom = CFzAlpha(end-lena+1:end,ind);
  tb = Alpha(end-lena+1:end,1) - 360;
  CFzFitData(2*lena*(ind-1)+1:2*lena*ind,1) = [tb;ta];
  CFzFitData(2*lena*(ind-1)+1:2*lena*ind,2) = [bottom;top];
end
CFzLinFit = polyfit(CFzFitData(:,1),CFzFitData(:,2),1);
CFzCoeff = CFzLinFit(end-1);

% Cm
CmFitData = zeros(2*lena*c+1,2);
for ind = 1:c
  top = CmAlpha(1:lena,ind);
  ta = Alpha(1:lena,1);
  bottom = CmAlpha(end-lena+1:end,ind);
  tb = Alpha(end-lena+1:end,1) - 360;
  CmFitData(2*lena*(ind-1)+1:2*lena*ind,1) = [tb;ta];
  CmFitData(2*lena*(ind-1)+1:2*lena*ind,2) = [bottom;top];
end
CmLinFit = polyfit(CmFitData(:,1),CmFitData(:,2),1);
CmCoeff = CmLinFit(end-1);

%% Save The Data %%
% Alpha Vector is Scaled!
Alpha = round(Alpha*MeshMult);
% Save Data
save('AerodynamicProperties.mat','mu','Alpha','ReCF','CFxAlpha','CFzAlpha','ReCm','CmAlpha','MeshMult')
save('ControlLinearizations.mat','CFxCoeff','CFzCoeff','CmCoeff')

%% Plotter %%
figure(1)
plot(Alpha,CFxAlpha)

toc
end

