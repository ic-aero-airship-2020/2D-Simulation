function [referenceMap,manualPath] = SetupSimulationEnvironment(complexity)
    switch complexity
        case 1
            load("exampleMaps.mat", "simpleMap");
            referenceMap = binaryOccupancyMap(simpleMap,1);
        case 2
            load("exampleMaps.mat", "complexMap");
            referenceMap = binaryOccupancyMap(complexMap,1);
        case 3
            actualLayout = imread('Map/Actual Layout.png');
            simplifiedLayout = imread('Map/Simplified 2D Layout.png');
            greyImage = rgb2gray(simplifiedLayout);
            bwImage = imbinarize(greyImage);
            referenceMap = binaryOccupancyMap(imcomplement(bwImage),35);
            
            figure('Name','Comparing layout');
            subplot(1,2,1);
            imshow(actualLayout);
            title("Actual Layout");
            subplot(1,2,2);
            show(referenceMap)
            title("Simplified Layout");

        otherwise
            load("exampleMaps.mat", "simpleMap");
            referenceMap = binaryOccupancyMap(simpleMap,1);
    end   
    
    switch complexity
       case 1
            manualPath = [4 6; 6.5 12.5; 4 22; 12 14; 22 22; 16 12; 20 10; 14 6; 22 3];
%             manualPath = [8.5 5.5; 8.5 11.5;2.5 9.5; 2.5 2.5; 5.5 2.5; 5.5 5.5];
        case 2
            manualPath = [4.5 3.5; 7.5 9.5; 16.5 9.5; 15.5 15.5; 2.5 15.5; 2.5 28.5; 46.5 28.5; 48.5 14.5; 40.5 18.5; 31.5 13.5; 32.5 9.5; 46.5 8.5; 46.5 4.5; 46.5 36.5];
        case 3
            manualPath =  [51.6000   54.8571
                           38.0857   62.9571
                           29.4857   62.6571
                           29.4857   53.0571
                            9.8286   53.0571
                            9.8286   45.2286
                           29.4857   45.2286
                           29.4857   24.5714
                           20.8857   24.5714
                           19.6571   14.7429
                           11.0571   14.7429
                           11.0571   19.6571
                           19.6571   19.6571
                           20.8857   24.5714
                            4.4571   24.5714];
        otherwise
            manualPath = [4 6; 6.5 12.5; 4 22; 12 14; 22 22; 16 12; 20 10; 14 6; 22 3];
    end
end