%MagData = out.MagData.Data;
MagData = readmatrix("Mag_noMagnet.csv");
%MagData = MagData(10:end, :); 
%MagData = MagData(:,1:end)

[A,b,expmfs] = magcal(MagData, 'auto');

MagData_cal = (MagData-b)*A;

figure(1)
plot3(MagData(:,1),MagData(:,2),MagData(:,3),'LineStyle','none','Marker','X','MarkerSize',8)
hold on
grid(gca,'on')
plot3(MagData_cal(:,1),MagData_cal(:,2),MagData_cal(:,3),'LineStyle','none','Marker', ...
            'o','MarkerSize',8,'MarkerFaceColor','r') 
axis equal
xlabel('uT')
ylabel('uT')
zlabel('uT')
legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
hold off
