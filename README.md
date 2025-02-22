# TSP Solver

A high-performance solver for the traveling salesman problem (TSP) in 3D and 2D space.

## Installation

1. Download the `.dll` file from the repository.
2. Ajoutez le fichier `.dll` à votre projet en tant que référence.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE.txt) file for details.

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