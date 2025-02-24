namespace TSP_Solver
{
    #region 2D
    /// <summary>
    /// The 2D solver
    /// </summary>
    public class TSPSolver2D
    {
        /// <summary>
        /// TSP points
        /// </summary>
        public List<City2> Cities { get; set; } = new();

        /// <summary>
        /// Usable distances
        /// </summary>
        public List<Distance2> Distances { get; set; } = new();

        /// <summary>
        /// 2D solver builder
        /// </summary>
        /// <param name="cities">TSP cities</param>
        public TSPSolver2D(List<City2> cities)
        {
            Cities = cities;

            for (int i = 0; i < Cities.Count; i++)
            {
                for (int j = i + 1; j < Cities.Count; j++)
                {
                    Distances.Add(new Distance2(Cities[i], Cities[j]));
                }
            }
        }

        /// <summary>
        /// The function for solving the problem in a 2D configuration
        /// </summary>
        /// <param name="threeOpt">If you use threeOpt</param>
        /// <returns>The best route found, its distance and the MST</returns>
        public (List<City2> route, double length, double MST) Solve(bool threeOpt)
        {
            List<City2> route = TSPSolver2DFunctions.Solve(Cities, Distances);
            TSPSolver2DFunctions.TwoOpt(ref route);

            if (threeOpt)
                TSPSolver2DFunctions.ThreeOpt(ref route);

            bool improved = true;

            while (improved)
            {
                improved = TSPSolver2DFunctions.OneOpt(ref route);
            }

            TSPSolver2DFunctions.TwoOpt(ref route);

            if (threeOpt)
                TSPSolver2DFunctions.ThreeOpt(ref route);

            double length = TSPSolver2DFunctions.CalculateTotalPathLength(route);

            double MST = TSPSolver2DFunctions.ComputeMSTLength(Cities, Distances).Item2;

            return (route, length, MST);
        }
    }

    /// <summary>
    /// The class contains the functions used by the 2D solver, so you don't need to use it
    /// </summary>
    public static class TSPSolver2DFunctions
    {
        /// <summary>
        /// The basic solver, which does most of the work
        /// </summary>
        /// <param name="cities">TSP cities</param>
        /// <param name="distances">Usable distances</param>
        /// <returns></returns>
        public static List<City2> Solve(List<City2> cities, List<Distance2> distances)
        {
            int numberOfCities = 0;
            numberOfCities = cities.Count;

            double MST = ComputeMSTLength(cities, distances).totalLength;

            List<Distance2> distancesOrdered = new List<Distance2>(distances.OrderBy(d => d.DistanceSquear));
            double Dmax = distancesOrdered.Last().DistanceSquear;
            List<Distance2> buffer = new List<Distance2>(distancesOrdered);

            List<Distance2> route = new List<Distance2>();
            UnionFind uf = new UnionFind(numberOfCities);

            #region Adaptive factor
            double meanEdgeLength = MST / (numberOfCities - 1);
            double variance = 0.0;

            foreach (var edge in distances)
            {
                variance += Math.Pow(edge.DistanceSquear - meanEdgeLength, 2);
            }

            variance /= (numberOfCities - 1);
            double standardDeviation = Math.Sqrt(variance);
            double adaptiveFactor = Math.Max(standardDeviation / meanEdgeLength, 0.05 + 0.1 * Math.Log(numberOfCities));

            adaptiveFactor *= 1 + (Math.Log(numberOfCities) / 10);
            #endregion

            #region Density factor
            List<City2> convexHull = GetConvexHull(cities);
            double convexHullArea = ComputeConvexHullArea(convexHull);
            double densityFactor = MST / (numberOfCities * Math.Sqrt(convexHullArea));

            densityFactor *= Math.Pow(numberOfCities / 10.0, 0.2);
            #endregion

            #region Hybrid factor
            double weight = Math.Min(numberOfCities / 1000.0, 1);
            double hybridFactor = adaptiveFactor * (1 - weight) + densityFactor * weight;

            double adjustmentFactor = 1 + Math.Log(numberOfCities) / 15;
            hybridFactor *= adjustmentFactor;
            #endregion

            double referenceFactor = Math.Max(MST, meanEdgeLength * numberOfCities);

            while (route.Count < numberOfCities && distancesOrdered.Count > 0)
            {
                Distance2 d = distancesOrdered.First();
                distancesOrdered.Remove(d);

                if (d.Second.TimeUsed >= 2 || d.First.TimeUsed >= 2)
                    continue;

                if (route.Count < numberOfCities - 1)
                {
                    if (buffer.Count > 0)
                    {
                        Dmax = buffer.Max(d => d.DistanceSquear);
                    }
                    else
                    {
                        Dmax = 0;
                    }

                    List<Distance2> caseOnebuffer = new List<Distance2>(buffer);
                    List<Distance2> caseTwobuffer = new List<Distance2>(buffer);

                    if (d.First.TimeUsed + 1 >= 2)
                    {
                        caseOnebuffer.RemoveAll(a => a.Contains(d.First) && a.DistanceSquear > d.DistanceSquear);
                    }

                    if (d.Second.TimeUsed + 1 >= 2)
                    {
                        caseOnebuffer.RemoveAll(a => a.Contains(d.Second) && a.DistanceSquear > d.DistanceSquear);
                    }

                    caseTwobuffer.Remove(d);

                    (bool, double) caseOneMST = ComputeMSTLength(cities, caseOnebuffer);
                    (bool, double) caseTwoMST = ComputeMSTLength(cities, caseTwobuffer);

                    #region Ratios
                    double ratioOne = (caseOneMST.Item2 / caseTwoMST.Item2) / MST;
                    double ratioTwo = (caseTwoMST.Item2 / caseOneMST.Item2) / MST;

                    double normalizedRatioOne = Math.Min(ratioOne, Math.Log(numberOfCities) * 0.5);
                    double normalizedRatioTwo = Math.Min(ratioTwo, Math.Log(numberOfCities) * 0.5);
                    #endregion

                    double magicNumberOne = 1 + (normalizedRatioOne * (numberOfCities * (numberOfCities - 1) / 2) * (2 * referenceFactor) * hybridFactor);
                    double magicNumberTwo = 1 + (normalizedRatioTwo * (numberOfCities * (numberOfCities - 1) / 2) * (2 * referenceFactor) * hybridFactor);

                    double adjustedOne = caseOneMST.Item2 * magicNumberOne;
                    double adjustedTwo = caseTwoMST.Item2 * magicNumberTwo;

                    bool caseOne = false;

                    if (caseOneMST.Item1 && (adjustedOne <= adjustedTwo || !caseTwoMST.Item1))
                        caseOne = true;

                    if (caseOne)
                    {
                        if (!uf.Union(d.First, d.Second))
                        {
                            continue;
                        }

                        route.Add(d);
                        d.First.TimeUsed++;
                        d.Second.TimeUsed++;

                        if (d.First.TimeUsed >= 2)
                        {
                            buffer.RemoveAll(a => a.Contains(d.First) && a.DistanceSquear > d.DistanceSquear);
                        }

                        if (d.Second.TimeUsed >= 2)
                        {
                            buffer.RemoveAll(a => a.Contains(d.Second) && a.DistanceSquear > d.DistanceSquear);
                        }
                    }
                    else
                    {
                        buffer.Remove(d);
                    }
                }
                else
                {
                    route.Add(d);
                    d.First.TimeUsed++;
                    d.Second.TimeUsed++;
                    break;
                }
            }

            List<City2> citiesTraveled = GetTravel(route);
            return citiesTraveled;
        }

        /// <summary>
        /// Retrieves the towns traversed by the path, 
        /// in the order of travel
        /// </summary>
        /// <param name="route">Path distances</param>
        /// <returns>
        /// List of cities visited, 
        /// in order of visit
        /// </returns>
        /// <exception cref="Exception">
        /// Appears if a distance that should have existed
        /// is not present in the path
        /// </exception>
        public static List<City2> GetTravel(List<Distance2> route)
        {
            List<Distance2> temp = new List<Distance2>(route);
            List<City2> result = new List<City2>();

            Distance2 first = temp.First();
            City2 start = first.First;
            City2 commune = first.Second;

            temp.Remove(first);
            result.Add(start);
            result.Add(commune);

            while (temp.Count > 0)
            {
                Distance2 d = temp.Find(t => t.Contains(commune));

                if (d == null)
                    throw new Exception("Distance qui n'existe pas");

                if (d.First == commune)
                {
                    commune = d.Second;
                }
                else
                {
                    commune = d.First;
                }

                result.Add(commune);
                temp.Remove(d);
            }

            return result;
        }

        /// <summary>
        /// Search for the MST with certain usable distances
        /// </summary>
        /// <param name="cities">TSP points</param>
        /// <param name="distances">Usable distances</param>
        /// <returns>
        /// True if the MST can be created, 
        /// and the length of the TSP
        /// </returns>
        public static (bool success, double totalLength) ComputeMSTLength(List<City2> cities, List<Distance2> distances)
        {
            if (cities == null || distances == null || cities.Count == 0)
                return (false, 0.0);

            UnionFind uf = new UnionFind(cities.Count);
            var sortedEdges = distances.OrderBy(d => d.DistanceSquear);

            double totalLength = 0.0;
            int edgesUsed = 0;

            foreach (var d in sortedEdges)
            {
                if (uf.Union(d.First, d.Second))
                {
                    totalLength += Math.Sqrt(d.DistanceSquear);
                    edgesUsed++;

                    if (edgesUsed == cities.Count - 1)
                        break;
                }
            }

            bool success = (edgesUsed == cities.Count - 1);
            return (success, totalLength);
        }

        /// <summary>
        /// Exchange 2 points to improve the road, 
        /// continues as long as the road is improved
        /// </summary>
        /// <param name="route">The path</param>
        public static void TwoOpt(ref List<City2> route)
        {
            int n = route.Count - 1;
            bool improvement = true;

            while (improvement)
            {
                improvement = false;
                for (int i = 1; i < n - 1; i++)
                {
                    for (int j = i + 1; j < n; j++)
                    {
                        if (ShouldSwap(route, i, j))
                        {
                            Swap(ref route, i, j);
                            improvement = true;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// If it's worth trading 2 points
        /// </summary>
        /// <param name="route">The path</param>
        /// <param name="i">First point index</param>
        /// <param name="j">Second point index</param>
        /// <returns>True if it's worth</returns>
        public static bool ShouldSwap(List<City2> route, int i, int j)
        {
            double distBefore = CalculateDistance(route[i - 1], route[i]) + CalculateDistance(route[j], route[j + 1]);
            double distAfter = CalculateDistance(route[i - 1], route[j]) + CalculateDistance(route[i], route[j + 1]);

            return distAfter < distBefore;
        }

        /// <summary>
        /// Exchange 2 points between them
        /// </summary>
        /// <param name="route">The path</param>
        /// <param name="i">First point index</param>
        /// <param name="j">Second point index</param>
        public static void Swap(ref List<City2> route, int i, int j)
        {
            while (i < j)
            {
                City2 temp = route[i];
                route[i] = route[j];
                route[j] = temp;
                i++;
                j--;
            }
        }

        /// <summary>
        /// Swap 3-point positions to improve the path, 
        /// continue until it's no longer useful
        /// </summary>
        /// <param name="route">The path</param>
        public static void ThreeOpt(ref List<City2> route)
        {
            bool improved = true;

            while (improved)
            {
                improved = false;
                for (int i = 0; i < route.Count - 2; i++)
                {
                    for (int j = i + 1; j < route.Count - 1; j++)
                    {
                        for (int k = j + 1; k < route.Count; k++)
                        {
                            double currentDistance = CalculateTotalPathLength(route);
                            var newRoute = Apply3OptSwap(route, i, j, k);
                            double newDistance = CalculateTotalPathLength(newRoute);

                            if (newDistance < currentDistance)
                            {
                                route = newRoute;
                                improved = true;
                            }
                        }
                    }
                }
            }
            return;
        }

        /// <summary>
        /// Apply an exchange between 3 points on a path
        /// </summary>
        /// <param name="route">The path</param>
        /// <param name="i">First point index</param>
        /// <param name="j">Second point index</param>
        /// <param name="k">Third point index</param>
        /// <returns>The new path</returns>
        public static List<City2> Apply3OptSwap(List<City2> route, int i, int j, int k)
        {
            var newRoute = new List<City2>(route);
            Swap(ref newRoute, i, j);
            Swap(ref newRoute, j, k);

            return newRoute;
        }

        /// <summary>
        /// Calculates the distance between 2 points
        /// </summary>
        /// <param name="city1">The first point</param>
        /// <param name="city2">The second point</param>
        /// <returns>The distance between these two points</returns>
        public static double CalculateDistance(City2 city1, City2 city2)
        {
            return Math.Sqrt(Math.Pow(city2.X - city1.X, 2) + Math.Pow(city2.Y - city1.Y, 2));
        }

        /// <summary>
        /// Calculates the total size of a path
        /// </summary>
        /// <param name="route">The path</param>
        /// <returns>The path size</returns>
        public static double CalculateTotalPathLength(List<City2> route)
        {
            double totalLength = 0.0;

            for (int i = 0; i < route.Count - 1; i++)
            {
                totalLength += CalculateDistance(route[i], route[i + 1]);
            }

            totalLength += CalculateDistance(route[route.Count - 1], route[0]);

            return totalLength;
        }

        /// <summary>
        /// Find all the distances of a path
        /// </summary>
        /// <param name="route">The path</param>
        /// <returns>All the distances found</returns>
        public static List<Distance2> GetDistancesInPath(List<City2> route)
        {
            List<Distance2> result = new List<Distance2>();
            for (int i = 0; i < route.Count - 1; i++)
            {
                result.Add(new Distance2(route[i], route[i + 1]));
            }

            Distance2 d = new Distance2(route[route.Count - 1], route[0]);
            if (d.DistanceSquear > 0)
                result.Add(d);

            return result;
        }

        /// <summary>
        /// Find the convex hull
        /// </summary>
        /// <param name="points">TSP points</param>
        /// <returns>The convex hull found</returns>
        public static List<City2> GetConvexHull(List<City2> points)
        {
            if (points.Count <= 1)
                return new List<City2>(points);

            points = points.OrderBy(p => p.X).ThenBy(p => p.Y).ToList();

            List<City2> hull = new List<City2>();

            foreach (var p in points)
            {
                while (hull.Count >= 2 && CrossProduct(hull[hull.Count - 2], hull[hull.Count - 1], p) <= 0)
                    hull.RemoveAt(hull.Count - 1);
                hull.Add(p);
            }

            int lowerCount = hull.Count + 1;
            for (int i = points.Count - 2; i >= 0; i--)
            {
                City2 p = points[i];
                while (hull.Count >= lowerCount && CrossProduct(hull[hull.Count - 2], hull[hull.Count - 1], p) <= 0)
                    hull.RemoveAt(hull.Count - 1);
                hull.Add(p);
            }

            hull.RemoveAt(hull.Count - 1);
            return hull;
        }

        /// <summary>
        /// Calculates the cross product between 3 points
        /// </summary>
        /// <param name="o">The first point</param>
        /// <param name="a">The second point</param>
        /// <param name="b">The third point</param>
        /// <returns>The result of the cross product</returns>
        public static double CrossProduct(City2 o, City2 a, City2 b)
        {
            return (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);
        }

        /// <summary>
        /// Calculates the area of convex hull
        /// </summary>
        /// <param name="hull">The convex hull</param>
        /// <returns>The convex hull area as a double</returns>
        public static double ComputeConvexHullArea(List<City2> hull)
        {
            if (hull.Count < 3)
                return 0;

            double area = 0.0;
            int n = hull.Count;

            for (int i = 0; i < n; i++)
            {
                City2 current = hull[i];
                City2 next = hull[(i + 1) % n];

                area += (current.X * next.Y) - (next.X * current.Y);
            }

            return Math.Abs(area) / 2.0;
        }

        /// <summary>
        /// Moves a point to its optimum position
        /// </summary>
        /// <param name="route">The path you want to improve</param>
        /// <returns>True if it has found a better location else false</returns>
        public static bool OneOpt(ref List<City2> route)
        {
            List<Distance2> distances = GetDistancesInPath(route).OrderByDescending(d => d.DistanceSquear).ToList();
            Dictionary<City2, byte> cities = new Dictionary<City2, byte>();

            Dictionary<City2, List<Distance2>> cityDistances = new Dictionary<City2, List<Distance2>>();

            foreach (var distance in distances)
            {
                if (!cityDistances.ContainsKey(distance.First))
                    cityDistances[distance.First] = new List<Distance2>();
                if (!cityDistances.ContainsKey(distance.Second))
                    cityDistances[distance.Second] = new List<Distance2>();

                cityDistances[distance.First].Add(distance);
                cityDistances[distance.Second].Add(distance);
            }

            bool find = false;
            bool improved = false;

            int i = 0;
            double length = CalculateTotalPathLength(route);

            City2 c = null;
            List<City2> result = new List<City2>(route);

            foreach (City2 city in route)
            {
                if (!cities.ContainsKey(city))
                    cities.Add(city, 0);
            }

            while (!find && i < distances.Count)
            {
                Distance2 d = distances[i];
                cities[d.First]++;
                cities[d.Second]++;

                if (cities[d.First] >= 2 || cities[d.Second] >= 2)
                {
                    find = true;

                    if (cities[d.First] >= 2 && cities[d.Second] < 2)
                    {
                        c = d.First;
                    }
                    else if (cities[d.First] < 2 && cities[d.Second] >= 2)
                    {
                        c = d.Second;
                    }
                    else
                    {
                        double first = cityDistances[d.First].Max(a => a.DistanceSquear);
                        double second = cityDistances[d.Second].Max(a => a.DistanceSquear);

                        c = first < second ? d.Second : d.First;
                    }
                }
                i++;
            }

            if (c != null)
            {
                int index = route.IndexOf(c);

                for (int j = 0; j < route.Count; j++)
                {
                    List<City2> newRoute = new List<City2>(route);
                    newRoute.Remove(c);
                    newRoute.Insert(j, c);

                    double l = CalculateTotalPathLength(newRoute);

                    if (l < length)
                    {
                        improved = true;
                        length = l;
                        result = newRoute;
                    }
                }

                route = result;
                return improved;
            }

            return false;
        }
    }

    /// <summary>
    /// Class representing 2D points
    /// </summary>
    public class City2 : ICity
    {
        /// <summary>
        /// The Id of the point
        /// </summary>
        public int Id { get; set; }

        /// <summary>
        /// X position
        /// </summary>
        public double X { get; set; }

        /// <summary>
        /// Y position
        /// </summary>
        public double Y { get; set; }

        /// <summary>
        /// The times the point has been used in the basic route
        /// </summary>
        public byte TimeUsed { get; set; }

        /// <summary>
        /// City2 builder
        /// </summary>
        /// <param name="x">X position</param>
        /// <param name="y">Y position</param>
        /// <param name="id">The Id of the point</param>
        public City2(double x, double y, int id)
        {
            X = x;
            Y = y;
            Id = id;
            TimeUsed = 0;
        }
    }

    /// <summary>
    /// Class used by the 2D solver, so you don't need to use it
    /// </summary>
    public class Distance2
    {
        /// <summary>
        /// The first point of the distance
        /// </summary>
        public City2 First { get; set; }

        /// <summary>
        /// The second point of the distance
        /// </summary>
        public City2 Second { get; set; }

        /// <summary>
        /// The distance between the two points at squear
        /// </summary>
        public double DistanceSquear { get; set; }

        /// <summary>
        /// Verify if the distance contains a point
        /// </summary>
        /// <param name="city">The point</param>
        /// <returns>True if the point appears in the distance</returns>
        public bool Contains(City2 city)
        {
            if (First == city || Second == city)
                return true;
            return false;
        }

        /// <summary>
        /// Verify if the distance is composed of the two points
        /// </summary>
        /// <param name="cityOne">The first point</param>
        /// <param name="cityTwo">The second point</param>
        /// <returns>True if the distance is composed of the two points</returns>
        public bool Contains(City2 cityOne, City2 cityTwo)
        {
            if (Contains(cityOne) && Contains(cityTwo))
                return true;
            return false;
        }

        /// <summary>
        /// Distance2 builder
        /// </summary>
        /// <param name="first">The first point of the distance</param>
        /// <param name="second">The second point of the distance</param>
        public Distance2(City2 first, City2 second)
        {
            First = first;
            Second = second;

            DistanceSquear = ((second.X - first.X) * (second.X - first.X)) + ((second.Y - first.Y) * (second.Y - first.Y));
        }
    }
    #endregion

    #region 3D
    /// <summary>
    /// The 3D solver
    /// </summary>
    public class TSPSolver3D
    {
        /// <summary>
        /// TSP points
        /// </summary>
        public List<City3> Cities { get; set; } = new();

        /// <summary>
        /// Usable distances
        /// </summary>
        public List<Distance3> Distances { get; set; } = new();

        /// <summary>
        /// The cache of usable distances
        /// </summary>
        public Dictionary<Tuple<City3, City3>, double> CachedDistances { get; set; } = new();

        /// <summary>
        /// The earth radius (only for geodesic 3D)
        /// </summary>
        public double Radius { get; set; } = 0;

        /// <summary>
        /// If the TSP is symmetric
        /// </summary>
        public bool Symmetric { get; set; } = true;

        /// <summary>
        /// 3D solver builder
        /// </summary>
        /// <param name="cities">Points of the path you want to create</param>
        /// <param name="geodesic">If you use geodesic 3D</param>
        /// <param name="radius">The radius of the earth</param>
        public TSPSolver3D(List<City3> cities, bool geodesic, double radius = 6371000)
        {
            Cities = cities;

            if (geodesic)
            {
                Radius = radius;

                for (int i = 0; i < Cities.Count; i++)
                {
                    for (int j = i + 1; j < Cities.Count; j++)
                    {
                        Distances.Add(new Distance3(Cities[i], Cities[j], Radius));
                        CachedDistances.Add(Tuple.Create(Distances.Last().First, Distances.Last().Second), Distances.Last().Distance);
                    }
                }
            }else
            {
                for (int i = 0; i < Cities.Count; i++)
                {
                    for (int j = i + 1; j < Cities.Count; j++)
                    {
                        Distances.Add(new Distance3(Cities[i], Cities[j]));
                        CachedDistances.Add(Tuple.Create(Distances.Last().First, Distances.Last().Second), Distances.Last().Distance);
                    }
                }
            }
        }

        /// <summary>
        /// 3D solver builder for cases where distances are required in creation (asymmetrical or incomplete cases)
        /// </summary>
        /// <param name="cities">Points of the path you want to create</param>
        /// <param name="distances">Distances that can be used</param>
        /// <param name="geodesic">If you use geodesic 3D</param>
        /// <param name="symmetric">If the TSP is symmetric</param>
        /// <param name="radius">The radius of the earth</param>
        public TSPSolver3D(List<City3> cities, List<Distance3> distances, bool geodesic, bool symmetric, double radius = 6371000)
        {
            Cities = cities;
            Distances = distances;

            if (geodesic)
                Radius = radius;

            Symmetric = symmetric;

            for (int i = 0; i < distances.Count; i++)
            {
                CachedDistances.Add(Tuple.Create(Distances[i].First, Distances[i].Second), Distances[i].Distance);
            }
        }


        /// <summary>
        /// The function for solving the problem in a 3D configuration
        /// </summary>
        /// <param name="threeOpt">If you want to use 3-opt</param>
        /// <param name="twoAndOneOpt">If you want to use 2-opt and 1-opt</param>
        /// <returns>The best route found, its distance and the MST</returns>
        public (List<City3> route, double length, double MST) Solve(bool threeOpt = false, bool twoAndOneOpt = false)
        {
            List<City3> route = TSPSolver3DFunctions.Solve(Cities, Distances);

            if (twoAndOneOpt)
                TSPSolver3DFunctions.TwoOpt(ref route, CachedDistances, Symmetric);

            if (threeOpt)
                TSPSolver3DFunctions.ThreeOpt(ref route, CachedDistances, Symmetric);

            bool improved = true;

            while (improved && twoAndOneOpt)
            {
                improved = TSPSolver3DFunctions.OneOpt(ref route, Distances, CachedDistances, Symmetric);
            }

            if (twoAndOneOpt)
                TSPSolver3DFunctions.TwoOpt(ref route, CachedDistances, Symmetric);

            if (threeOpt)
                TSPSolver3DFunctions.ThreeOpt(ref route, CachedDistances, Symmetric);

            double length = TSPSolver3DFunctions.CalculateTotalPathLength(route, CachedDistances, Symmetric).Item2;

            double MST = TSPSolver3DFunctions.ComputeMSTLength(Cities, Distances).Item2;

            return (route, length, MST);
        }
    }

    /// <summary>
    /// The class contains the functions used by the 3D solver, so you don't need to use it
    /// </summary>
    public static class TSPSolver3DFunctions
    {
        /// <summary>
        /// The basic solver, 
        /// which does most of the work
        /// </summary>
        /// <param name="cities">TSP points</param>
        /// <param name="distances">Usable distances</param>
        /// <returns>The path found</returns>
        /// <exception cref="Exception">
        /// Appears if it is impossible to make the path 
        /// with just the usable distances
        /// </exception>
        public static List<City3> Solve(List<City3> cities, List<Distance3> distances)
        {
            int numberOfCities = 0;
            numberOfCities = cities.Count;

            double MST = ComputeMSTLength(cities, distances).totalLength;
            bool pathPossible = IsGraphConnected(cities, distances);

            if (!pathPossible)
                throw new Exception("Not enough distance to make a path");

            List<Distance3> distancesOrdered = new List<Distance3>(distances.OrderBy(d => d.Distance));

            List<Distance3> buffer = new List<Distance3>(distancesOrdered);

            List<Distance3> route = new List<Distance3>();
            UnionFind uf = new UnionFind(numberOfCities);

            #region Adaptive factor
            double meanEdgeLength = MST / (numberOfCities - 1);
            double variance = 0.0;

            foreach (var edge in distances)
            {
                variance += Math.Pow(edge.Distance - meanEdgeLength, 2);
            }

            variance /= (numberOfCities - 1);
            double standardDeviation = Math.Sqrt(variance);
            double adaptiveFactor = Math.Max(standardDeviation / meanEdgeLength, 0.05 + 0.1 * Math.Log(numberOfCities));

            adaptiveFactor *= 1 + (Math.Log(numberOfCities) / 10);
            #endregion

            #region Density factor
            List<Face> convexHull = GetConvexHull(cities);
            double convexHullArea = ComputeConvexHullArea(convexHull);

            double densityFactor = MST / (numberOfCities * Math.Sqrt(convexHullArea));

            densityFactor *= Math.Pow(numberOfCities / 10.0, 0.2);
            #endregion

            #region Hybrid factor
            double weight = Math.Min(numberOfCities / 1000.0, 1);
            double hybridFactor = adaptiveFactor * (1 - weight) + densityFactor * weight;

            double adjustmentFactor = 1 + Math.Log(numberOfCities) / 15;
            hybridFactor *= adjustmentFactor;
            #endregion

            double referenceFactor = Math.Max(MST, meanEdgeLength * numberOfCities);

            while (route.Count < numberOfCities && distancesOrdered.Count > 0)
            {
                Distance3 d = distancesOrdered.First();
                distancesOrdered.Remove(d);

                if (d.Second.TimeUsed >= 2 || d.First.TimeUsed >= 2)
                    continue;

                if (route.Count < numberOfCities - 1)
                {
                    List<Distance3> caseOnebuffer = new List<Distance3>(buffer);
                    List<Distance3> caseTwobuffer = new List<Distance3>(buffer);

                    if (d.First.TimeUsed + 1 >= 2)
                    {
                        caseOnebuffer.RemoveAll(a => a.Contains(d.First) && a.Distance > d.Distance);
                    }

                    if (d.Second.TimeUsed + 1 >= 2)
                    {
                        caseOnebuffer.RemoveAll(a => a.Contains(d.Second) && a.Distance > d.Distance);
                    }

                    caseTwobuffer.Remove(d);

                    double caseOneMST = ComputeMSTLength(cities, caseOnebuffer).totalLength;
                    double caseTwoMST = ComputeMSTLength(cities, caseTwobuffer).totalLength;

                    bool caseOneValid = IsGraphConnected(cities, caseOnebuffer);
                    bool caseTwoValid = IsGraphConnected(cities, caseTwobuffer);

                    #region Ratios
                    double ratioOne = (caseOneMST / caseTwoMST) / MST;
                    double ratioTwo = (caseTwoMST / caseOneMST) / MST;

                    double normalizedRatioOne = Math.Min(ratioOne, Math.Log(numberOfCities) * 0.5);
                    double normalizedRatioTwo = Math.Min(ratioTwo, Math.Log(numberOfCities) * 0.5);
                    #endregion

                    double magicNumberOne = 1 + (normalizedRatioOne * (numberOfCities * (numberOfCities - 1) / 2) * (2 * referenceFactor) * hybridFactor);
                    double magicNumberTwo = 1 + (normalizedRatioTwo * (numberOfCities * (numberOfCities - 1) / 2) * (2 * referenceFactor) * hybridFactor);

                    double adjustedOne = caseOneMST * magicNumberOne;
                    double adjustedTwo = caseTwoMST * magicNumberTwo;

                    bool caseOne = false;

                    if (caseOneValid && (adjustedOne <= adjustedTwo || !caseTwoValid))
                        caseOne = true;

                    if (caseOne)
                    {
                        if (!uf.Union(d.First, d.Second))
                        {
                            continue;
                        }

                        route.Add(d);
                        d.First.TimeUsed++;
                        d.Second.TimeUsed++;

                        if (d.First.TimeUsed >= 2)
                        {
                            buffer.RemoveAll(a => a.Contains(d.First) && a.Distance > d.Distance);
                        }

                        if (d.Second.TimeUsed >= 2)
                        {
                            buffer.RemoveAll(a => a.Contains(d.Second) && a.Distance > d.Distance);                    
                        }
                    }
                    else
                    {
                        buffer.Remove(d);
                    }
                }
                else
                {
                    route.Add(d);
                    d.First.TimeUsed++;
                    d.Second.TimeUsed++;
                    break;
                }
            }

            List<City3> citiesTraveled = GetTravel(route);
            return citiesTraveled;
        }

        /// <summary>
        /// Retrieves the towns traversed by the path, 
        /// in the order of travel
        /// </summary>
        /// <param name="route">Path distances</param>
        /// <returns>
        /// List of cities visited, 
        /// in order of visit
        /// </returns>
        /// <exception cref="Exception">
        /// Appears if a distance that should have existed
        /// is not present in the path
        /// </exception>
        public static List<City3> GetTravel(List<Distance3> route)
        {
            List<Distance3> temp = new List<Distance3>(route);
            List<City3> result = new List<City3>();

            Distance3 first = temp.First();
            City3 start = first.First;
            City3 commune = first.Second;

            temp.Remove(first);
            result.Add(start);
            result.Add(commune);

            while (temp.Count > 0)
            {
                Distance3 d = temp.Find(t => t.Contains(commune));

                if (d == null)
                    throw new Exception("Distance which does not exist");

                if (d.First == commune)
                {
                    commune = d.Second;
                }
                else
                {
                    commune = d.First;
                }

                result.Add(commune);
                temp.Remove(d);
            }

            return result;
        }

        /// <summary>
        /// Check if the graph can be connected
        /// </summary>
        /// <param name="cities">TSP points</param>
        /// <param name="distances">Usable distances</param>
        /// <returns></returns>
        public static bool IsGraphConnected(List<City3> cities, List<Distance3> distances)
        {
            UnionFind union = new UnionFind(cities.Count);

            foreach (var d in distances)
            {
                union.Union(d.First, d.Second);
            }

            return union.IsFullyConnected;
        }

        /// <summary>
        /// Search for the MST with certain usable distances
        /// </summary>
        /// <param name="cities">TSP points</param>
        /// <param name="distances">Usable distances</param>
        /// <returns>
        /// True if the MST can be created,
        /// and the length of the TSP
        /// </returns>
        public static (bool success, double totalLength) ComputeMSTLength(List<City3> cities, List<Distance3> distances)
        {
            if (cities == null || distances == null || cities.Count == 0)
                return (false, 0.0);

            UnionFind uf = new UnionFind(cities.Count);
            var sortedEdges = distances.OrderBy(d => d.Distance);

            double totalLength = 0.0;
            int edgesUsed = 0;

            foreach (var d in sortedEdges)
            {
                if (uf.Union(d.First, d.Second))
                {
                    totalLength += d.Distance;
                    edgesUsed++;

                    if (edgesUsed == cities.Count - 1)
                        break;
                }
            }

            bool success = edgesUsed == cities.Count - 1;
            return (success, totalLength);
        }

        /// <summary>
        /// Exchange 2 points to improve the road, 
        /// continues as long as the road is improved
        /// </summary>
        /// <param name="route">The path</param>
        /// <param name="cachedDistances">The cache of usable distances</param>
        /// <param name="symmetric">If the TSP is symmetric</param>
        public static void TwoOpt(ref List<City3> route, Dictionary<Tuple<City3, City3>, double> cachedDistances, bool symmetric)
        {
            int n = route.Count - 1;
            bool improvement = true;

            while (improvement)
            {
                improvement = false;
                for (int i = 1; i < n - 1; i++)
                {
                    for (int j = i + 1; j < n; j++)
                    {
                        if (ShouldSwap(route, i, j, cachedDistances, symmetric))
                        {
                            Swap(ref route, i, j);
                        }
                    }
                }
            }

            return;
        }

        /// <summary>
        /// If it's worth trading 2 points
        /// </summary>
        /// <param name="route">The path</param>
        /// <param name="i">The first point index</param>
        /// <param name="j">The second point index</param>
        /// <param name="cachedDistances">The cache of usable distances</param>
        /// <param name="symmetric">If the TSP is symmetric</param>
        /// <returns>True if it's worth</returns>
        public static bool ShouldSwap(List<City3> route, int i, int j, Dictionary<Tuple<City3, City3>, double> cachedDistances, bool symmetric)
        {
            if (i <= 0 || j >= route.Count - 1)
                return false;

            (bool find1, double newDist1) = CalculateDistance(route[i - 1], route[j], cachedDistances, symmetric);
            (bool find2, double newDist2) = CalculateDistance(route[i], route[j + 1], cachedDistances, symmetric);

            (bool find3, double oldDist1) = CalculateDistance(route[i - 1], route[i], cachedDistances, symmetric);
            (bool find4, double oldDist2) = CalculateDistance(route[j], route[j + 1], cachedDistances, symmetric);
            
            if (!(find1 && find2 && find3 && find4))
                return false;

            double distBefore = oldDist1 + oldDist2;
            double distAfter = newDist1 + newDist2;

            return distAfter < distBefore;
        }

        /// <summary>
        /// Exchange 2 points between them
        /// </summary>
        /// <param name="route">The path</param>
        /// <param name="i">The first point index</param>
        /// <param name="j">The second point index</param>
        public static void Swap(ref List<City3> route, int i, int j)
        {
            while (i < j)
            {
                City3 temp = route[i];
                route[i] = route[j];
                route[j] = temp;
                i++;
                j--;
            }
        }

        /// <summary>
        /// Swap 3-point positions to improve the path, 
        /// continue until it's no longer useful
        /// </summary>
        /// <param name="route">The path</param>
        /// <param name="cachedDistances">The cache of usable distances</param>
        /// <param name="symmetric">If the TSP is symmetric</param>
        public static void ThreeOpt(ref List<City3> route, Dictionary<Tuple<City3, City3>, double> cachedDistances, bool symmetric)
        {
            bool improved = true;
            double currentDistance = CalculateTotalPathLength(route, cachedDistances, symmetric).Item2;

            while (improved)
            {
                improved = false;
                for (int i = 0; i < route.Count - 2; i++)
                {
                    for (int j = i + 1; j < route.Count - 1; j++)
                    {
                        for (int k = j + 1; k < route.Count; k++)
                        {
                            var newRoute = Apply3OptSwap(route, i, j, k);
                            (bool v, double newDistance) newDist = CalculateTotalPathLength(newRoute, cachedDistances, symmetric);

                            if (newDist.newDistance < currentDistance && newDist.v)
                            {
                                route = newRoute;
                                currentDistance = newDist.newDistance;
                                improved = true;
                            }
                        }
                    }
                }
            }
            return;
        }

        /// <summary>
        /// Apply an exchange between 3 points on a path
        /// </summary>
        /// <param name="route">The path</param>
        /// <param name="i">First point index</param>
        /// <param name="j">Second point index</param>
        /// <param name="k">Third point index</param>
        /// <returns>The new path</returns>
        public static List<City3> Apply3OptSwap(List<City3> route, int i, int j, int k)
        {
            var newRoute = new List<City3>(route);
            Swap(ref newRoute, i, j);
            Swap(ref newRoute, j, k);

            return newRoute;
        }

        /// <summary>
        /// Calculates the distance between 2 points
        /// </summary>
        /// <param name="city1">The first point</param>
        /// <param name="city2">The second point</param>
        /// <param name="cachedDistances">The cache of usable distances</param>
        /// <param name="symmetric">If the TSP is symmetric</param>
        /// <returns>
        /// True if the distance between these two points exists, 
        /// and the distance between these two points
        /// </returns>
        public static (bool find, double length) CalculateDistance(City3 city1, City3 city2, Dictionary<Tuple<City3, City3>, double> cachedDistances, bool symmetric)
        {
            Tuple<City3, City3> key = symmetric ?  (city1.Id < city2.Id ? Tuple.Create(city1, city2) : Tuple.Create(city2, city1)) 
                : Tuple.Create(city1, city2);

            if (cachedDistances.TryGetValue(key, out double cachedDistance))
            {
                return (true, cachedDistance);
            }

            return (false, 0);
        }

        /// <summary>
        /// Calculates the total size of a path
        /// </summary>
        /// <param name="route">The path</param>
        /// <param name="cachedDistances">The cache of usable distances</param>
        /// <param name="symmetric">If the TSP is symmetric</param>
        /// <returns>
        /// True if the path consists only of existing distances,
        /// and the path size
        /// </returns>
        public static (bool, double) CalculateTotalPathLength(List<City3> route, Dictionary<Tuple<City3, City3>, double> cachedDistances, bool symmetric)
        {
            double totalLength = 0.0;
            bool valid = true;

            for (int i = 0; i < route.Count - 1; i++)
            {
                double length = CalculateDistance(route[i], route[i + 1], cachedDistances, symmetric).Item2;
                if (length >= 0)
                    totalLength += length;
                else
                    valid = false;
            }

            return (valid, totalLength);
        }

        /// <summary>
        /// Find all the distances of a path
        /// </summary>
        /// <param name="route">The path</param>
        /// <param name="distances">Usable distances</param>
        /// <param name="symmetric">If the TSP is symmetric</param>
        /// <returns>All the distances found</returns>
        public static List<Distance3> GetDistancesInPath(List<City3> route, List<Distance3> distances, bool symmetric)
        {
            List<Distance3> result = new List<Distance3>();
            for (int i = 0; i < route.Count - 1; i++)
            {
                if (route[i].Id != route[i + 1].Id)
                {
                    if (!symmetric)
                    {
                        List<Distance3> find = distances.FindAll(a => a.Is(route[i], route[i + 1])).ToList();

                        if (find.Count > 0)
                        {
                            result.Add(find.First());
                        }
                    }else
                    {
                        List<Distance3> find = distances.FindAll(a => a.Contains(route[i], route[i + 1])).ToList();

                        if (find.Count > 0)
                        {
                            result.Add(find.First());
                        }
                    }
                }
            }

            return result;
        }

        /// <summary>
        /// Find the approximate convex hull
        /// </summary>
        /// <param name="points">TSP points</param>
        /// <returns>The approximate convex hull found</returns>
        public static List<Face> GetConvexHull(List<City3> points)
        {
            if (points.Count < 4) return new List<Face>();

            City3 minX = points.OrderBy(p => p.Position.X).First();
            City3 maxX = points.OrderBy(p => p.Position.X).Last();
            City3 minY = points.OrderBy(p => p.Position.Y).First();
            City3 maxY = points.OrderBy(p => p.Position.Y).Last();
            City3 minZ = points.OrderBy(p => p.Position.Z).First();
            City3 maxZ = points.OrderBy(p => p.Position.Z).Last();

            List<Face> hull = new List<Face>();

            hull.Add(new Face(minX, minY, minZ));
            hull.Add(new Face(minX, minY, maxZ));
            hull.Add(new Face(minX, maxY, minZ));
            hull.Add(new Face(minX, maxY, maxZ));
            hull.Add(new Face(maxX, minY, minZ));
            hull.Add(new Face(maxX, minY, maxZ));
            hull.Add(new Face(maxX, maxY, minZ));
            hull.Add(new Face(maxX, maxY, maxZ));

            return hull;
        }

        /// <summary>
        /// Calculates the area of a convex hull
        /// </summary>
        /// <param name="hull">The convex hull</param>
        /// <returns>The area of the convex hull as a double</returns>
        public static double ComputeConvexHullArea(List<Face> hull)
        {
            double area = 0.0;
            foreach (var face in hull)
            {
                Vector3 ab = face.B.Position - face.A.Position;
                Vector3 ac = face.C.Position - face.A.Position;
                area += 0.5 * Vector3.Cross(ab, ac).Norme();
            }
            return area;
        }

        /// <summary>
        /// Moves a point to its optimum position
        /// </summary>
        /// <param name="route">The path you want to improve</param>
        /// <param name="allDistances">Usable distances</param>
        /// <param name="cachedDistances">The cache of usable distances</param>
        /// <param name="symmetric">If the TSP is symmetric</param>
        /// <returns>True if it has found a better location else false</returns>
        public static bool OneOpt(ref List<City3> route, List<Distance3> allDistances, Dictionary<Tuple<City3, City3>, double> cachedDistances, bool symmetric)
        {
            List<Distance3> distances = GetDistancesInPath(route, allDistances, symmetric).OrderByDescending(d => d.Distance).ToList();
            Dictionary<City3, byte> cities = new Dictionary<City3, byte>();

            Dictionary<City3, List<Distance3>> cityDistances = new Dictionary<City3, List<Distance3>>();

            foreach (var distance in distances)
            {
                if (!cityDistances.ContainsKey(distance.First))
                    cityDistances[distance.First] = new List<Distance3>();
                if (!cityDistances.ContainsKey(distance.Second))
                    cityDistances[distance.Second] = new List<Distance3>();

                cityDistances[distance.First].Add(distance);
                cityDistances[distance.Second].Add(distance);
            }

            bool find = false;
            bool improved = false;

            int i = 0;
            double length = CalculateTotalPathLength(route, cachedDistances, symmetric).Item2;

            City3 c = null;
            List<City3> result = new List<City3>(route);

            foreach (City3 city in route)
            {
                if (!cities.ContainsKey(city))
                    cities.Add(city, 0);
            }

            while (!find && i < distances.Count)
            {
                Distance3 d = distances[i];
                cities[d.First]++;
                cities[d.Second]++;

                if (cities[d.First] >= 2 || cities[d.Second] >= 2)
                {
                    find = true;

                    if (cities[d.First] >= 2 && cities[d.Second] < 2)
                    {
                        c = d.First;
                    }
                    else if (cities[d.First] < 2 && cities[d.Second] >= 2)
                    {
                        c = d.Second;
                    }
                    else
                    {
                        double first = cityDistances[d.First].Max(a => a.Distance);
                        double second = cityDistances[d.Second].Max(a => a.Distance);

                        c = first < second ? d.Second : d.First;
                    }
                }
                i++;
            }

            if (c != null)
            {
                int index = route.IndexOf(c);

                for (int j = 0; j < route.Count; j++)
                {
                    List<City3> newRoute = new List<City3>(route);
                    newRoute.Remove(c);
                    newRoute.Insert(j, c);

                    (bool v, double l) vl = CalculateTotalPathLength(newRoute, cachedDistances, symmetric);

                    if (vl.l < length && vl.v)
                    {
                        improved = true;
                        length = vl.l;
                        result = newRoute;
                    }
                }

                route = result;
                return improved;
            }

            return false;
        }
    }

    /// <summary>
    /// Class representing 3D points
    /// </summary>
    public class City3: ICity
    {
        /// <summary>
        /// The Id of the point to identify it
        /// </summary>
        public int Id { get; set; }

        /// <summary>
        /// The point's coordinates
        /// </summary>
        public Vector3 Position { get; set; }

        /// <summary>
        /// The times the point appears in the path distances created by the basic solver
        /// </summary>
        public byte TimeUsed { get; set; } = 0;

        /// <summary>
        /// The radius of the earth
        /// </summary>
        public double EarthRadius { get; set; } = 6371000;

        /// <summary>
        /// City3 builder for cartesian 3D
        /// </summary>
        /// <param name="position">The vector3 which contains the positions of the point</param>
        /// <param name="id">The Id of the point</param>
        public City3(Vector3 position, int id)
        {
            Position = position;
            Id = id;
        }

        /// <summary>
        /// City3 builder for geodesic 3D
        /// </summary>
        /// <param name="latitude">The latitude of the point in radians</param>
        /// <param name="longitude">The longitude of the point in radians</param>
        /// <param name="altitude">The altitude of the point in meters</param>
        /// <param name="id">The Id of the point</param>
        /// <param name="radius">The radius of the earth</param>
        public City3(double latitude, double longitude, double altitude, int id, double radius = 6371000)
        {
            double X = (radius + altitude) * Math.Cos(latitude) * Math.Cos(longitude);
            double Y = (radius + altitude) * Math.Cos(latitude) * Math.Sin(longitude);
            double Z = (radius + altitude) * Math.Sin(latitude);
            Id = id;
            Position = new Vector3(X, Y, Z);
            EarthRadius = radius;
        }

        /// <summary>
        /// Converts a point's cartesian coordinates into geodesic coordinates
        /// </summary>
        /// <returns>Return point's geodesic coordinates</returns>
        public (double latitude, double longitude, double altitude) CartesianToGeodetic()
        {
            double longitude = Math.Atan2(Position.Y, Position.X);

            double latitudinalDistance = Math.Sqrt(Position.X * Position.X + Position.Y * Position.Y);
            double latitude = Math.Atan2(Position.Z, latitudinalDistance);

            double altitude = Math.Sqrt(Position.X * Position.X + Position.Y * Position.Y + Position.Z * Position.Z) - EarthRadius;

            return (latitude, longitude, altitude);
        }
    }

    /// <summary>
    /// Class used by the 3D solver, so you don't need to use it
    /// </summary>
    public class Distance3
    {
        /// <summary>
        /// The first point of the distance
        /// </summary>
        public City3 First { get; set; }

        /// <summary>
        /// The second point of the distance
        /// </summary>
        public City3 Second { get; set; }

        /// <summary>
        /// The distance between the first point and the second point
        /// </summary>
        public double Distance { get; set; }

        /// <summary>
        /// Verify if the distance contains the point
        /// </summary>
        /// <param name="city">The point to test</param>
        /// <returns>True if the distance contains the point else false</returns>
        public bool Contains(City3 city)
        {
            if (First == city || Second == city)
                return true;
            return false;
        }

        /// <summary>
        /// Verify if the distance is composed of the two points
        /// </summary>
        /// <param name="cityOne">One of the two points to test</param>
        /// <param name="cityTwo">One of the two points to test</param>
        /// <returns>True if the distance contains the two points else false</returns>
        public bool Contains(City3 cityOne, City3 cityTwo)
        {
            if (Contains(cityOne) && Contains(cityTwo))
                return true;
            return false;
        }

        /// <summary>
        /// Simple Distance3 builder
        /// </summary>
        /// <param name="first">One of the points of the distance</param>
        /// <param name="second">The other point of the distance</param>
        public Distance3(City3 first, City3 second)
        {
            if (first.Id < second.Id)
            {
                First = first;
                Second = second;
            }
            else
            {
                First = second;
                Second = first;
            }

            Distance = Math.Sqrt(Math.Pow(second.Position.X - first.Position.X, 2) + Math.Pow(second.Position.Y - first.Position.Y, 2) + Math.Pow(second.Position.Z - first.Position.Z, 2));
        }

        /// <summary>
        /// Distance3 builder for the geodesic 3D
        /// </summary>
        /// <param name="first">The first point of the distance</param>
        /// <param name="second">The second point of the distance</param>
        /// <param name="radius">The earth radius</param>
        public Distance3(City3 first, City3 second, double radius)
        {
            if (first.Id < second.Id)
            {
                First = first;
                Second = second;
            }
            else
            {
                First = second;
                Second = first;
            }

            Distance = GeodesicDistance(first.Position, second.Position, radius);
        }

        /// <summary>
        /// Distance3 builder for the case where you have the distance
        /// </summary>
        /// <param name="distance">The distance between the start point and the end point</param>
        /// <param name="first">The start point</param>
        /// <param name="second">The end point</param>
        public Distance3(double distance, City3 first, City3 second)
        {
            First = first;
            Second = second;
            Distance = distance;
        }
        private static double GeodesicDistance(Vector3 p1, Vector3 p2, double radius)
        {
            double dotProduct = Vector3.Dot(p1, p2);
            double normA = p1.Norme();
            double normB = p2.Norme();

            double cosTheta = dotProduct / (normA * normB);
            double theta = Math.Acos(cosTheta);

            return radius * theta;
        }

        /// <summary>
        /// If the points formed exactly the distance
        /// </summary>
        /// <param name="first">The start point</param>
        /// <param name="second">The end point</param>
        /// <returns>
        /// True if the start point of the distance is the start point test 
        /// and if the end point of the distance is the end point of the distance
        /// </returns>
        public bool Is(City3 first, City3 second)
        {
            return (first == First && second == Second);
        }
    }

    /// <summary>
    /// Class used by some solver functions, so you don't need it
    /// </summary>
    public class Face
    {
        /// <summary>
        /// Vertices of the face
        /// </summary>
        public City3 A, B, C;

        /// <summary>
        /// The normal of the face
        /// </summary>
        public Vector3 Normal;

        /// <summary>
        /// Face builder
        /// </summary>
        /// <param name="a">One of the corner points</param>
        /// <param name="b">One of the corner points</param>
        /// <param name="c">One of the corner points</param>
        public Face(City3 a, City3 b, City3 c)
        {
            A = a;
            B = b;
            C = c;
            Normal = Vector3.Cross(B.Position - A.Position, C.Position - A.Position);
            Normal = Vector3.Normalize(Normal);
        }
    }

    /// <summary>
    /// As a simple 3D vector, you don't need to know its functions or use it in any of your methods
    /// </summary>
    public struct Vector3
    {
        /// <summary>
        /// X position
        /// </summary>
        public double X { get; set; }

        /// <summary>
        /// Y position
        /// </summary>
        public double Y { get; set; }

        /// <summary>
        /// Z position
        /// </summary>
        public double Z { get; set; }

        /// <summary>
        /// Vector3 builder
        /// </summary>
        /// <param name="X">X position</param>
        /// <param name="Y">Y position</param>
        /// <param name="Z">Z position</param>
        public Vector3(double X, double Y, double Z)
        {
            this.X = X; this.Y = Y; this.Z = Z;
        }

        /// <summary>
        /// Calculate the norm of the vector
        /// </summary>
        /// <returns>The norme of the vector</returns>
        public double Norme()
        {
            return Math.Sqrt(Math.Pow(X, 2) + Math.Pow(Y, 2) + Math.Pow(Z, 2));
        }

        /// <summary>
        /// Calculate the vector product between 2 vectors
        /// </summary>
        /// <param name="v1">The first vector</param>
        /// <param name="v2">The second vector</param>
        /// <returns>The cross product</returns>
        public static Vector3 Cross(Vector3 v1, Vector3 v2)
        {
            return new Vector3(
                v1.Y * v2.Z - v1.Z * v2.Y,
                v1.Z * v2.X - v1.X * v2.Z,
                v1.X * v2.Y - v1.Y * v2.X
            );
        }

        /// <summary>
        /// Subtract one vector from another
        /// </summary>
        /// <param name="v1">The first vector</param>
        /// <param name="v2">The second vector</param>
        /// <returns>The result vector</returns>
        public static Vector3 operator -(Vector3 v1, Vector3 v2)
        {
            double X = v1.X - v2.X;
            double Y = v1.Y - v2.Y;
            double Z = v1.Z - v2.Z;

            return new Vector3(X, Y, Z);
        }

        /// <summary>
        /// Divide a vector by a number
        /// </summary>
        /// <param name="v">The vector</param>
        /// <param name="scalar">The divisor</param>
        /// <returns>The vector divided</returns>
        public static Vector3 operator /(Vector3 v, double scalar)
        {
            if (scalar == 0)
                return v;

            return new Vector3(v.X / scalar, v.Y / scalar, v.Z / scalar);
        }

        /// <summary>
        /// Calculate the scalar product between two vectors
        /// </summary>
        /// <param name="vector1">The first vector</param>
        /// <param name="vector2">The second vector</param>
        /// <returns>The scalar product calculated</returns>
        public static double Dot(Vector3 vector1, Vector3 vector2)
        {
            return (vector1.X * vector2.X) + (vector1.Y * vector2.Y) + (vector1.Z * vector2.Z);
        }

        /// <summary>
        /// Normalize vector
        /// </summary>
        /// <param name="v">The vector to be normalized</param>
        /// <returns>The vector normalized</returns>
        public static Vector3 Normalize(Vector3 v)
        {
            double norme = v.Norme();
            if (norme > 0)
            {
                return v / norme;
            }
            return v;
        }
    }

    #endregion

    /// <summary>
    /// Interface used by points
    /// </summary>
    public interface ICity
    {
        /// <summary>
        /// The Id of the point
        /// </summary>
        int Id { get; set; }

        /// <summary>
        /// The times the point has been used in the basic route
        /// </summary>
        byte TimeUsed { get; set; }
    }

    /// <summary>
    /// Class used by the solver, so you don't need it
    /// </summary>
    public class UnionFind
    {
        private int[] parent;
        private int count;

        /// <summary>
        /// UnionFind builder
        /// </summary>
        /// <param name="n">The number of cities</param>
        public UnionFind(int n)
        {
            parent = Enumerable.Range(0, n).ToArray();
            count = n;
        }

        private int Find(int i)
        {
            if (parent[i] != i)
                parent[i] = Find(parent[i]);
            return parent[i];
        }

        /// <summary>
        /// Add a link between the 2 points
        /// </summary>
        /// <param name="i">First point</param>
        /// <param name="j">Second point</param>
        /// <returns>
        /// False if the points were already joined,
        /// otherwise true
        /// </returns>
        public bool Union(ICity i, ICity j)
        {
            int rootI = Find(i.Id);
            int rootJ = Find(j.Id);

            if (rootI == rootJ)
                return false;

            parent[rootJ] = rootI;
            count--;

            return true;
        }

        /// <summary>
        /// If all the points are connected
        /// </summary>
        public bool IsFullyConnected => count == 1;
    }
}