namespace TSP_Solver
{
    #region 2D
    public class TSPSolver2D
    {
        public List<City2> Cities { get; set; } = new();
        public List<Distance2> Distances { get; set; } = new();

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

        public (List<City2> route, double length, double MST) Solve(bool threeOpt)
        {
            (List<City2> route, double length) result = TSPSolver2DFunctions.Solve(Cities, Distances);
            TSPSolver2DFunctions.TwoOpt(ref result.route);

            if (threeOpt)
                TSPSolver2DFunctions.ThreeOpt(ref result.route);

            bool improved = true;

            while (improved)
            {
                improved = TSPSolver2DFunctions.OneOpt(ref result.route);
            }

            TSPSolver2DFunctions.TwoOpt(ref result.route);

            if (threeOpt)
                TSPSolver2DFunctions.ThreeOpt(ref result.route);

            double MST = TSPSolver2DFunctions.ComputeMSTLength(Cities, Distances).Item2;

            return (result.route, result.length, MST);
        }
    }

    public static class TSPSolver2DFunctions
    {
        public static (List<City2> route, double length) Solve(List<City2> cities, List<Distance2> distances)
        {
            int numberOfCities = 0;
            numberOfCities = cities.Count;

            double MST = ComputeMSTLength(cities, distances).totalLength;

            List<Distance2> distancesOrdered = new List<Distance2>(distances.OrderBy(d => d.DistanceSquear));
            double Dmax = distancesOrdered.Last().DistanceSquear;
            List<Distance2> buffer = new List<Distance2>(distancesOrdered);

            List<Distance2> route = new List<Distance2>();
            UnionFind uf = new UnionFind(numberOfCities);

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

                    #region Magic numbers
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

                    #region Ratios
                    double ratioOne = (caseOneMST.Item2 / caseTwoMST.Item2) / MST;
                    double ratioTwo = (caseTwoMST.Item2 / caseOneMST.Item2) / MST;

                    double normalizedRatioOne = Math.Min(ratioOne, Math.Log(numberOfCities) * 0.5);
                    double normalizedRatioTwo = Math.Min(ratioTwo, Math.Log(numberOfCities) * 0.5);
                    #endregion

                    double referenceFactor = Math.Max(MST, meanEdgeLength * numberOfCities);
                    double magicNumberOne = 1 + (normalizedRatioOne * (numberOfCities * (numberOfCities - 1) / 2) * (2 * referenceFactor) * hybridFactor);
                    double magicNumberTwo = 1 + (normalizedRatioTwo * (numberOfCities * (numberOfCities - 1) / 2) * (2 * referenceFactor) * hybridFactor);
                    #endregion

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

            double length = 0;
            foreach (Distance2 distance in route)
            {
                length += Math.Sqrt(distance.DistanceSquear);
            }

            List<City2> citiesTraveled = GetTravel(route);
            return (citiesTraveled, length);
        }

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
                            TwoOptSwap(route, i, j);
                            improvement = true;
                        }
                    }
                }
            }
        }

        public static bool ShouldSwap(List<City2> route, int i, int j)
        {
            double distBefore = CalculateDistance(route[i - 1], route[i]) + CalculateDistance(route[j], route[j + 1]);
            double distAfter = CalculateDistance(route[i - 1], route[j]) + CalculateDistance(route[i], route[j + 1]);

            return distAfter < distBefore;
        }

        public static void TwoOptSwap(List<City2> route, int i, int j)
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

        public static List<City2> Apply3OptSwap(List<City2> route, int i, int j, int k)
        {
            var newRoute = new List<City2>(route);
            TwoOptSwap(newRoute, i, j);
            TwoOptSwap(newRoute, j, k);

            return newRoute;
        }

        public static double CalculateDistance(City2 city1, City2 city2)
        {
            return Math.Sqrt(Math.Pow(city2.X - city1.X, 2) + Math.Pow(city2.Y - city1.Y, 2));
        }

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

        private static double CrossProduct(City2 o, City2 a, City2 b)
        {
            return (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);
        }

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
                    TwoOptSwap(newRoute, i, j);

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

    public class City2 : City
    {
        public int Id { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
        public byte TimeUsed { get; set; }

        public City2(double x, double y, int id)
        {
            X = x;
            Y = y;
            Id = id;
            TimeUsed = 0;
        }
    }

    public class Distance2
    {
        public City2 First { get; set; }
        public City2 Second { get; set; }
        public double DistanceSquear { get; set; }

        public bool Contains(City2 city)
        {
            if (First == city || Second == city)
                return true;
            return false;
        }

        public bool Contains(City2 cityOne, City2 cityTwo)
        {
            if (Contains(cityOne) && Contains(cityTwo))
                return true;
            return false;
        }

        public Distance2(City2 first, City2 second)
        {
            First = first;
            Second = second;

            DistanceSquear = ((second.X - first.X) * (second.X - first.X)) + ((second.Y - first.Y) * (second.Y - first.Y));
        }
    }
    #endregion

    #region 3D
    public class TSPSolver3D
    {
        public List<City3> Cities { get; set; } = new();
        public List<Distance3> Distances { get; set; } = new();

        public TSPSolver3D(List<City3> cities)
        {
            Cities = cities;

            for (int i = 0; i < Cities.Count; i++)
            {
                for (int j = i + 1; j < Cities.Count; j++)
                {
                    Distances.Add(new Distance3(Cities[i], Cities[j]));
                }
            }
        }

        public (List<City3> route, double length, double MST) Solve(bool threeOpt)
        {
            (List<City3> route, double length) result = TSPSolver3DFunctions.Solve(Cities, Distances);
            TSPSolver3DFunctions.TwoOpt(ref result.route);

            if (threeOpt)
            {
                TSPSolver3DFunctions.ThreeOpt(ref result.route);
            }

            bool improved = true;

            while (improved)
            {
                improved = TSPSolver3DFunctions.OneOpt(ref result.route);
            }

            TSPSolver3DFunctions.TwoOpt(ref result.route);

            if (threeOpt)
            {
                TSPSolver3DFunctions.ThreeOpt(ref result.route);
            }

            double MST = TSPSolver3DFunctions.ComputeMSTLength(Cities, Distances).Item2;

            return (result.route, result.length, MST);
        }
    }

    public static class TSPSolver3DFunctions
    {
        public static (List<City3> route, double length) Solve(List<City3> cities, List<Distance3> distances)
        {
            int numberOfCities = 0;
            numberOfCities = cities.Count;

            double MST = ComputeMSTLength(cities, distances).totalLength;

            List<Distance3> distancesOrdered = new List<Distance3>(distances.OrderBy(d => d.DistanceSquear));
            double Dmax = distancesOrdered.Last().DistanceSquear;
            List<Distance3> buffer = new List<Distance3>(distancesOrdered);

            List<Distance3> route = new List<Distance3>();
            UnionFind uf = new UnionFind(numberOfCities);

            while (route.Count < numberOfCities && distancesOrdered.Count > 0)
            {
                Distance3 d = distancesOrdered.First();
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

                    List<Distance3> caseOnebuffer = new List<Distance3>(buffer);
                    List<Distance3> caseTwobuffer = new List<Distance3>(buffer);

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

                    #region Magic numbers
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
                    List<Face> convexHull = GetApproximatedConvexHull(cities);
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

                    #region Ratios
                    double ratioOne = (caseOneMST.Item2 / caseTwoMST.Item2) / MST;
                    double ratioTwo = (caseTwoMST.Item2 / caseOneMST.Item2) / MST;

                    double normalizedRatioOne = Math.Min(ratioOne, Math.Log(numberOfCities) * 0.5);
                    double normalizedRatioTwo = Math.Min(ratioTwo, Math.Log(numberOfCities) * 0.5);
                    #endregion

                    double referenceFactor = Math.Max(MST, meanEdgeLength * numberOfCities);
                    double magicNumberOne = 1 + (normalizedRatioOne * (numberOfCities * (numberOfCities - 1) / 2) * (2 * referenceFactor) * hybridFactor);
                    double magicNumberTwo = 1 + (normalizedRatioTwo * (numberOfCities * (numberOfCities - 1) / 2) * (2 * referenceFactor) * hybridFactor);
                    #endregion

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

            double length = 0;
            foreach (Distance3 distance in route)
            {
                length += Math.Sqrt(distance.DistanceSquear);
            }

            List<City3> citiesTraveled = GetTravel(route);
            return (citiesTraveled, length);
        }

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

        public static (bool success, double totalLength) ComputeMSTLength(List<City3> cities, List<Distance3> distances)
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

        public static void TwoOpt(ref List<City3> route)
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
                            TwoOptSwap(route, i, j);
                            improvement = true;
                        }
                    }
                }
            }
        }

        public static bool ShouldSwap(List<City3> route, int i, int j)
        {
            double distBefore = CalculateDistance(route[i - 1], route[i]) + CalculateDistance(route[j], route[j + 1]);
            double distAfter = CalculateDistance(route[i - 1], route[j]) + CalculateDistance(route[i], route[j + 1]);

            return distAfter < distBefore;
        }

        public static void TwoOptSwap(List<City3> route, int i, int j)
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

        public static void ThreeOpt(ref List<City3> route)
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

        public static List<City3> Apply3OptSwap(List<City3> route, int i, int j, int k)
        {
            var newRoute = new List<City3>(route);
            TwoOptSwap(newRoute, i, j);
            TwoOptSwap(newRoute, j, k);

            return newRoute;
        }

        public static double CalculateDistance(City3 city1, City3 city2)
        {
            return Math.Sqrt(Math.Pow(city2.Position.X - city1.Position.X, 2) + Math.Pow(city2.Position.Y - city1.Position.Y, 2) + Math.Pow(city2.Position.Z - city1.Position.Z, 2));
        }

        public static double CalculateTotalPathLength(List<City3> route)
        {
            double totalLength = 0.0;

            for (int i = 0; i < route.Count - 1; i++)
            {
                totalLength += CalculateDistance(route[i], route[i + 1]);
            }

            totalLength += CalculateDistance(route[route.Count - 1], route[0]);

            return totalLength;
        }

        public static List<Distance3> GetDistancesInPath(List<City3> route)
        {
            List<Distance3> result = new List<Distance3>();
            for (int i = 0; i < route.Count - 1; i++)
            {
                result.Add(new Distance3(route[i], route[i + 1]));
            }

            Distance3 d = new Distance3(route[route.Count - 1], route[0]);
            if (d.DistanceSquear > 0)
                result.Add(d);

            return result;
        }

        public static List<Face> GetConvexHull(List<City3> points)
        {
            if (points.Count < 4) return new List<Face>();

            City3 minX = points.OrderBy(p => p.Position.X).First();
            City3 maxX = points.OrderBy(p => p.Position.X).Last();
            City3 minY = points.OrderBy(p => p.Position.Y).First();
            City3 maxY = points.OrderBy(p => p.Position.Y).Last();
            City3 minZ = points.OrderBy(p => p.Position.Z).First();
            City3 maxZ = points.OrderBy(p => p.Position.Z).Last();

            List<City3> baseTetra = new List<City3> { minX, maxX, minY, maxY };

            List<Face> hull = new List<Face>
    {
        new Face(baseTetra[0], baseTetra[1], baseTetra[2]),
        new Face(baseTetra[0], baseTetra[1], baseTetra[3]),
        new Face(baseTetra[0], baseTetra[2], baseTetra[3]),
        new Face(baseTetra[1], baseTetra[2], baseTetra[3])
    };

            foreach (var p in points)
            {
                if (baseTetra.Contains(p)) continue;

                List<Face> visibleFaces = hull.Where(f => Vector3.Dot(f.Normal, p.Position - f.A.Position) > 0).ToList();

                if (visibleFaces.Count == 0) continue;

                hull.RemoveAll(f => visibleFaces.Contains(f));

                foreach (var face in visibleFaces)
                {
                    hull.Add(new Face(face.A, face.B, p));
                    hull.Add(new Face(face.B, face.C, p));
                    hull.Add(new Face(face.C, face.A, p));
                }
            }

            return hull;
        }

        public static List<Face> GetApproximatedConvexHull(List<City3> points)
        {
            if (points.Count < 4) return new List<Face>();

            City3 minX = points.OrderBy(p => p.Position.X).First();
            City3 maxX = points.OrderBy(p => p.Position.X).Last();
            City3 minY = points.OrderBy(p => p.Position.Y).First();
            City3 maxY = points.OrderBy(p => p.Position.Y).Last();
            City3 minZ = points.OrderBy(p => p.Position.Z).First();
            City3 maxZ = points.OrderBy(p => p.Position.Z).Last();

            List<City3> extremePoints = new List<City3> { minX, maxX, minY, maxY, minZ, maxZ };

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

        public static bool OneOpt(ref List<City3> route)
        {
            List<Distance3> distances = GetDistancesInPath(route).OrderByDescending(d => d.DistanceSquear).ToList();
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
            double length = CalculateTotalPathLength(route);

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
                    List<City3> newRoute = new List<City3>(route);
                    TwoOptSwap(newRoute, i, j);

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

    public class City3 : City
    {
        public int Id { get; set; }
        public Vector3 Position { get; set; }
        public byte TimeUsed { get; set; }

        public City3(Vector3 position, int id)
        {
            Position = position;
            Id = id;
            TimeUsed = 0;
        }
    }

    public class Distance3
    {
        public City3 First { get; set; }
        public City3 Second { get; set; }
        public double DistanceSquear { get; set; }

        public bool Contains(City3 city)
        {
            if (First == city || Second == city)
                return true;
            return false;
        }

        public bool Contains(City3 cityOne, City3 cityTwo)
        {
            if (Contains(cityOne) && Contains(cityTwo))
                return true;
            return false;
        }

        public Distance3(City3 first, City3 second)
        {
            First = first;
            Second = second;

            DistanceSquear = ((second.Position.X - first.Position.X) * (second.Position.X - first.Position.X)) + ((second.Position.Y - first.Position.Y) * (second.Position.Y - first.Position.Y)) + ((second.Position.Z - first.Position.Z) * (second.Position.Z - first.Position.Z));
        }
    }

    public struct Vector3
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }

        public Vector3(double X, double Y, double Z)
        {
            this.X = X; this.Y = Y; this.Z = Z;
        }

        public double Norme()
        {
            return Math.Sqrt(Math.Pow(X, 2) + Math.Pow(Y, 2) + Math.Pow(Z, 2));
        }

        public static Vector3 Cross(Vector3 v1, Vector3 v2)
        {
            return new Vector3(
                v1.Y * v2.Z - v1.Z * v2.Y,
                v1.Z * v2.X - v1.X * v2.Z,
                v1.X * v2.Y - v1.Y * v2.X
            );
        }

        public static Vector3 operator -(Vector3 v1, Vector3 v2)
        {
            double X = v1.X - v2.X;
            double Y = v1.Y - v2.Y;
            double Z = v1.Z - v2.Z;

            return new Vector3(X, Y, Z);
        }

        public static Vector3 operator /(Vector3 v, double scalar)
        {
            if (scalar == 0)
                return v;

            return new Vector3(v.X / scalar, v.Y / scalar, v.Z / scalar);
        }

        public static double Dot(Vector3 vector1, Vector3 vector2)
        {
            return (vector1.X * vector2.X) + (vector1.Y * vector2.Y) + (vector1.Z * vector2.Z);
        }

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

    public class Face
    {
        public City3 A, B, C;
        public Vector3 Normal;

        public Face(City3 a, City3 b, City3 c)
        {
            A = a;
            B = b;
            C = c;
            Normal = Vector3.Cross(B.Position - A.Position, C.Position - A.Position);
            Normal = Vector3.Normalize(Normal);
        }

        public bool IsEqual(Face other)
        {
            return (A == other.A && B == other.B && C == other.C) ||
                   (A == other.A && B == other.C && C == other.B) ||
                   (A == other.B && B == other.A && C == other.C) ||
                   (A == other.B && B == other.C && C == other.A) ||
                   (A == other.C && B == other.A && C == other.B) ||
                   (A == other.C && B == other.B && C == other.A);
        }

        public bool IsVisible(City3 point)
        {
            return Vector3.Dot(Normal, point.Position - A.Position) > 0;
        }
    }

    #endregion

    public interface City
    {
        int Id { get; set; }
        byte TimeUsed { get; set; }
    }

    public class UnionFind
    {
        private int[] parent;
        public UnionFind(int n)
        {
            parent = Enumerable.Range(0, n).ToArray();
        }

        public int Find(int i)
        {
            if (parent[i] != i)
                parent[i] = Find(parent[i]);
            return parent[i];
        }

        public bool Union(City i, City j)
        {
            int rootI = Find(i.Id);
            int rootJ = Find(j.Id);
            if (rootI == rootJ)
                return false;
            parent[rootJ] = rootI;
            return true;
        }
    }
}
