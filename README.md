# TSP Solver

A high-performance solver for the traveling salesman problem (TSP) in 3D and 2D space. It doesn't yet support geodesic 3D, and I don't know if I'd have the courage to do so.

## Installation

1. Download the `.dll` file from the repository.
2. Ajoutez le fichier `.dll` à votre projet en tant que référence.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE.txt) file for details.

## Known limitations

This system starts to be slow around 350 points (it takes a few seconds to find a path) and may (probably) not always find the best result.

## Classes and Functions

1. **2D classes and functions**:

	**A. TSPSolver2D**: The class to use for solving a TSP configuration. Functions:

		a. Solve: The function for solving the problem. It returns the best path found, the path length and the MST. If you want to retrieve only one of the 3 results, you can put behind the call: 

			-'.route' to retrieve the path, 
			-'.length' to retrieve the path length, 
			-'.MST' to retrieve the MST.

	**B. TSPSolver2DFunctions**: The class containing all the functions used by the solver. A static class. Functions:

		a. Solve: The function does most of the work: it takes as input the cities and distances to be used, and outputs the path that seems to be the best for these distances.
	
		b. GetTravel: The function for finding the sequence of points making up the path from the path distances.

		c. ComputeMSTLength: The function for retrieving the MST from cities and a list of usable distances.
	
		d. TwoOpt: The 2-opt algorithm is used to improve the path found.

		e. ShouldSwap: The function for determining whether 2 points in the path should be reversed. Used by 2-opt.

		f. TwoOptswap: The function used to exchange the positions of 2 cities in the path.

		g. ThreeOpt: The 3-opt algorithm, used if desired to improve the result.

		h. Apply3OptSwap: The function for exchanging 3 points in the path.
	
		i. CalculateDistance: The function for calculating the distance between 2 points.

		j. CalculateTotalPathLength: The function that calculates the total length of a path.

		k. GetDistancesInPath: The function that returns the list of distances on a path.

		l. GetConvexHull: The function returning the hull convex containing all points.

		m. CrossProduct: The function returns the cross product between 3 cities.
	
		n. ComputeConvexHullArea: The function that returns the area of the convex hull.

		o. OneOpt: An algorithm that moves the point with the greatest distances to the most efficient location.

	**C. City2**: Represents a point.

	**D. Distance2**: Represents a distance between 2 points. This class contains the squared distance between the 2 points, as well as the 2 points themselves.


2. **3D classes and functions**:

	**A. TSPSolver3D**: The class to use for solving a TSP configuration. Functions:

		a. Solve: The function for solving the problem. It returns the best path found, the path length and the MST. If you want to retrieve only one of the 3 results, you can put behind the call: 

			-'.route' to retrieve the path, 
			-'.length' to retrieve the path length, 
			-'.MST' to retrieve the MST.

	**B. TSPSolver3DFunctions**: The class containing all the functions used by the solver. A static class. Functions:

		a. Solve: The function does most of the work: it takes as input the cities and distances to be used, and outputs the path that seems to be the best for these distances.
	
		b. GetTravel: The function for finding the sequence of points making up the path from the path distances.

		c. ComputeMSTLength: The function for retrieving the MST from cities and a list of usable distances.
	
		d. TwoOpt: The 2-opt algorithm is used to improve the path found.

		e. ShouldSwap: The function for determining whether 2 points in the path should be reversed. Used by 2-opt.

		f. TwoOptswap: The function used to exchange the positions of 2 cities in the path.

		g. ThreeOpt: The 3-opt algorithm, used if desired to improve the result.

		h. Apply3OptSwap: The function for exchanging 3 points in the path.
	
		i. CalculateDistance: The function for calculating the distance between 2 points.

		j. CalculateTotalPathLength: The function that calculates the total length of a path.

		k. GetDistancesInPath: The function that returns the list of distances on a path.

		l. GetConvexHull: The function returning the hull convex containing all points.
	
		m. ComputeConvexHullArea: The function that returns the area of the convex hull.

		n. OneOpt: An algorithm that moves the point with the greatest distances to the most efficient location.

	**C. City3**: Represents a point.

	**D. Distance3**: Represents a distance between 2 points. This class contains the squared distance between the 2 points, as well as the 2 points themselves.

	**E. Vector3**: A simple 3D vector used in the points. Functions:

		a. Norme: Returns the vector norm. A non-static function.
	
		b. Cross: Return the cross product between 2 vectors. A static function.

		c. -: Subtracts a vector from another vector. Static function. An operator.

		d. /: Divide a vector by a number. Operator. An operator.

		e. Dot: Returns the scalar product between 2 vectors. A static function.

		f. Normalize: Normalize a vector. A static function.

	**F. Face**: Used for the convex hull, a simple face with three edges.


3. **2D and 3D classes and functions**:

	**A. City**: The interface used by the various classes serving as points.

	**B. UnionFind**: The class used to check whether a loop will be created. Functions:
	
		a. Find: Returns the furthest parent (in terms of links) of index point i.

		b. Union: Creates a union between 2 points and returns false if they were already linked. 


## Usage

Here are two examples of use in C# :

1:

```csharp
using TSP_Solver;

Console.WriteLine("Number of cities:");
int numberOfCities = int.Parse(Console.ReadLine());

Console.WriteLine("Max X; Y and Z:");
long maxXY = long.Parse(Console.ReadLine());

Console.WriteLine("Min X; Y and Z:");
long minXY = long.Parse(Console.ReadLine());

List<City3> cities = new List<City3>();
List<Distance3> distances = new List<Distance3>();

for (int i = 0; i < numberOfCities; i++)
{
    cities.Add(new City3(new Vector3(Random.Shared.NextInt64(minXY, maxXY), Random.Shared.NextInt64(minXY, maxXY), Random.Shared.NextInt64(minXY, maxXY)), i));
}

foreach (var city in cities)
{
    Console.WriteLine($"City {city.Id}; Positions: X: {city.Position.X} Y: {city.Position.Y} Z: {city.Position.Z}");
}

TSPSolver3D solver = new TSPSolver3D(cities);

(List<City3>, double, double) result = solver.Solve(false);
List<City3> citiesTraveled = result.Item1;
double length = result.Item2;
double MST = result.Item3;

Console.Write("Route: ");
for (int i = 0; i < citiesTraveled.Count; i++)
{
    Console.Write(citiesTraveled[i].Id);
    if (i < citiesTraveled.Count - 1)
    {
        Console.Write(" -> ");
    }else
    {
        Console.WriteLine();
    }
}

Console.WriteLine("Route length: " + length);
Console.WriteLine("MST: " + MST);

double ratio = length / MST;
Console.WriteLine("Ratio: " + ratio);

2:

```csharp
using TSP_Solver;

Console.WriteLine("Number of cities:");
int numberOfCities = int.Parse(Console.ReadLine());

Console.WriteLine("Max X and Y:");
long maxXY = long.Parse(Console.ReadLine());

Console.WriteLine("Min X and Y:");
long minXY = long.Parse(Console.ReadLine());

List<City2> cities = new List<City2>();
List<Distance2> distances = new List<Distance2>();

for (int i = 0; i < numberOfCities; i++)
{
    cities.Add(new City2(Random.Shared.NextInt64(minXY, maxXY), Random.Shared.NextInt64(minXY, maxXY), i));
}

foreach (var city in cities)
{
    Console.WriteLine($"City {city.Id}; Positions: X: {city.X} Y: {city.Y}");
}

TSPSolver2D solver = new TSPSolver2D(cities);

(List<City2>, double, double) result = solver.Solve(false);
List<City2> citiesTraveled = result.Item1;
double length = result.Item2;
double MST = result.Item3;

Console.Write("Route: ");
for (int i = 0; i < citiesTraveled.Count; i++)
{
    Console.Write(citiesTraveled[i].Id);
    if (i < citiesTraveled.Count - 1)
    {
        Console.Write(" -> ");
    }else
    {
        Console.WriteLine();
    }
}

Console.WriteLine("Route length: " + length);
Console.WriteLine("MST: " + MST);

double ratio = length / MST;
Console.WriteLine("Ratio: " + ratio);