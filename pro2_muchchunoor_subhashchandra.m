function pro2_muchchunoor_subhashchandra(mobility_model, velocity, pausing_time, k)
    %   user inputs: 1)mobilityModel: 0 for RWP, 1 for PPR, 2)velocity: 1 to 5 m/s
    %   3)pausingTime: 0 to 10 secs, 4)k: 2 or 3 (Number of dummy locations in the virtual rectangle)

    networkSize = [1000, 1000]; % 1000 x 1000 m^2 network size
    num_of_random_dest = 4;
    numDummyLocations=k-1;
    % initialize the drone position
    drone_pos =[0,0];

    figure;
    hold on;
    xlim([0, networkSize(1)]);
    ylim([0, networkSize(2)]);
    grid on;

    if mobility_model == 0
        mobilityModelName = 'Random Waypoint Mobility';
        pathMarker = '*';     
    else
        mobilityModelName = 'Privacy-Preserving Random Mobility';
        pathMarker = 'o';
    end
    title(mobilityModelName);
    % Plot initial drone position
    plot(drone_pos(1), drone_pos(2), pathMarker, 'MarkerSize', 10,'Color','r', 'LineWidth', 1.5);

    totalFlyingDistance = 0;
    totalFlyingTime = 0;
    
    %checking if the given mobility model is within 0 or 1
     if mobility_model~=1 && mobility_model~=0
        fprintf('Please enter the Valid Mobility model either 0-for RWP or 1-for PPR\n');
     else
        for iteration = 1:num_of_random_dest % Main simulation loop
	    
           if mobility_model==0
	       
                 % Choose a random destination within the network
                destination = [randi(networkSize(1)), randi(networkSize(2))];
        
                % Calculate distance to the destination
                distanceToDest = norm(destination - drone_pos);
        
                % Calculate flying time based on velocity
                flyingTime = distanceToDest / velocity;
        
                % Update performance measures
                totalFlyingDistance = totalFlyingDistance + distanceToDest;
                totalFlyingTime = totalFlyingTime + flyingTime;
                
                plot(destination(1),destination(2),'*','MarkerSize', 10,'Color','r','LineWidth', 1.5);
                % Fly towards the destination
                t = 0;
                while t <= flyingTime
                    t = t + 2;    
                    interpPos = drone_pos + (destination - drone_pos) * (t / flyingTime);
                    plot(interpPos(1), interpPos(2), '.','MarkerSize', 6,'Color','g');
                    %plot(interpPos(1), interpPos(2), pathMarker, 'MarkerSize', 10, 'LineWidth', 1.5);
                    drawnow;
                end
        
                % Update drone position
                drone_pos = destination;
        
                % Pause at the destination for pausingTime seconds
                pause(pausing_time);
        
           end
    
           if mobility_model == 1
			    % Privacy-Preserving Random (PPR) model
			    
			    % Initialize total distance through dummy locations
			    destination =  [randi(networkSize(1)), randi(networkSize(2))]; % Generate a random destination within the network
    
			    % Calculate the distance and time required to reach the destination
			    distance = norm(destination - drone_pos);
                time = distance / velocity;
			    plot(destination(1),destination(2),'o'); 
			    rectDiagonal = [drone_pos; destination]; % Draw a virtual rectangle with a diagonal from the current position to the destination
			    rectangle('Position', [min(rectDiagonal), max(rectDiagonal) - min(rectDiagonal)], 'EdgeColor', 'g','LineWidth', 1.5);
    
			    % Generate dummy locations within the rectangle
                dummyLocations = [];
                for i = 1:numDummyLocations + 1
                    dummyLocation = [randi([min(rectDiagonal(:, 1)), max(rectDiagonal(:, 1))]), ...
                        randi([min(rectDiagonal(:, 2)), max(rectDiagonal(:, 2))])];
                    dummyLocations = [dummyLocations; dummyLocation];
                end
    
                distancesToDummyLocations = vecnorm(dummyLocations - drone_pos, 2, 2);
        
                % Sortint the dummy locations based on their distances from the drone position
                [~, sortedIndices] = sort(distancesToDummyLocations);
                dummyLocations = dummyLocations(sortedIndices, :);
    
			    % Fly towards the destination through all the dummy locations
                fprintf('Drones flying distance and time through the specified dummy locations\nafter reaching the random destination: %d\n', iteration);
			    for i = 1:numDummyLocations + 1
				    if i == 1
					    path = drone_pos + (dummyLocations(i, :) - drone_pos) .* linspace(0, 1, round(time / numDummyLocations) + 1).';
				    elseif i == numDummyLocations+1
                        path = dummyLocations(i-1,:) + (destination - dummyLocations(i-1,:)) .* linspace(0, 1, round(time/numDummyLocations)+1).';                
                    else
					    path = dummyLocations(i - 1, :) + (dummyLocations(i, :) - dummyLocations(i - 1, :)) .* linspace(0, 1, round(time / numDummyLocations) + 1).';
                    end
    
				    plot(path(:, 1), path(:, 2), '.', 'Color', 'b');
				    hold on;
				    drawnow;
    
				    % Calculate distance to the next waypoint
				    if i < numDummyLocations + 1
					    distanceToNextDummyLoc = norm(dummyLocations(i + 1, :) - dummyLocations(i, :));
				    else
					    distanceToNextDummyLoc = norm(destination - dummyLocations(i, :));
				    end
    
				    % Calculate flying time to the next waypoint
				    flyingTimeToNextDummyLoc = distanceToNextDummyLoc / velocity;
    
				     %Show the drone movement along the dummy locations
				    for t = 0:1:flyingTimeToNextDummyLoc					
					    interpPos = path(1, :) + (path(end, :) - path(1, :)) * (t / flyingTimeToNextDummyLoc);
    
					    % Plot drone location at each time interval
					    plot(interpPos(1), interpPos(2), 'o', 'MarkerSize', 5, 'Color', '#be6400');
					    hold on;
					    drawnow;
				    end
				    
				    % Update totalFlyingDistance and totalFlyingTime
				    totalFlyingDistance = totalFlyingDistance + distanceToNextDummyLoc;
    
                    totalFlyingTime = totalFlyingTime + flyingTimeToNextDummyLoc;
                    fprintf('Flying Distance: %.2f meters\n', totalFlyingDistance);
                    fprintf('Flying Time: %.2f seconds\n', totalFlyingTime);
    
                    pause(2);
			    end
    
			    % Mark the drone's location at the final destination
			    plot(drone_pos(1), drone_pos(2), 'O','MarkerSize', 10,'Color','r','LineWidth', 1.5);
			    hold on;
    
			    drone_pos = destination; % Update the drone's position
			    pause(pausing_time); % Pause for the specified pausing time
    
		    end
           
        end
    
        hold off;
    
        % Print the Flying Distance and Flying Time of the Drone mobility
        fprintf('\n-----------Final results----------:\n');
        fprintf('Model: %s\n', mobilityModelName);
        fprintf('Total Flying Distance of the Drone: %.2f meters\n', totalFlyingDistance);
        fprintf('Total Flying Time of the Drone: %.2f seconds\n', totalFlyingTime);
     end
end
