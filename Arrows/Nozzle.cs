using System;
using System.Collections.Generic;
using System.Linq;
using devDept.Eyeshot;
using devDept.Eyeshot.Entities;
using devDept.Geometry;

namespace SectioningProblem
{
    public class Nozzle
    {
        public Brep Shell;
        public Brep Neck;
        public Brep Pad;
        public List<Brep> Welds = new List<Brep>();
        public Point3D Center;

        public Nozzle(CaseArgs args)
        {
            Welds.Clear();
            RunCase(args);
        }

        public void RunCase(CaseArgs selectedCase)
        {

            // shell that we're attaching the neck too
            Shell = selectedCase.Shell;

            // length that we'll extrude our neck
            var extrudeLength = selectedCase.ExternalLength + selectedCase.InternalLength + selectedCase.ShellThickness;


            // normal to define our plane with various angles (all cases I defined are orthogonal)
            var normal = new Vector3D(Math.Cos(selectedCase.Theta) * Math.Sin(selectedCase.Beta),
                Math.Sin(selectedCase.Theta) * Math.Sin(selectedCase.Beta), Math.Cos(selectedCase.Beta));
            normal.Normalize();
            var flushPlane = new Plane(selectedCase.ReferencePoint, normal);
            var extrudePlane = flushPlane.Clone() as Plane;
            extrudePlane.Translate(normal * (selectedCase.ExternalLength + selectedCase.ShellThickness)); 
            extrudePlane.Flip();
            var hillSideVector = (selectedCase.HillsidePlane == Plane.YZ) ? flushPlane.AxisY * selectedCase.Hillside : flushPlane.AxisX * selectedCase.Hillside;

            /*
             *
             * Defining our neck
             *
             */
            var circle = new Circle(extrudePlane, extrudePlane.Origin, selectedCase.NeckRadius);
            var face = circle.OffsetToRegion(selectedCase.NeckThickness, 0.0001, true);
            var faceNorm = new Vector3D(extrudePlane.Equation.X, extrudePlane.Equation.Y, extrudePlane.Equation.Z);
            // region we'll use to cut the shell and reinforcement pad later
            var cutRegion = new Region(circle.Offset(selectedCase.NeckThickness, faceNorm));

            // This handles the case where we have a Translation for the neck along the surface of the shell
            face.Translate(hillSideVector);
            cutRegion.Translate(hillSideVector);
            Neck = face.ExtrudeAsBrep(extrudeLength);
            var uncutNeck = Neck.Clone() as Brep;

            var intersectionCurves = new List<ICurve>();
            GetIntersectionCurves(Shell, Neck, out var tmpCurves, out var intersectedSurfaces);
            tmpCurves = UtilityEx.GetConnectedCurves(tmpCurves, 0.0001).ToList();
            var ordered = (selectedCase.HillsidePlane == Plane.XY) ? 
                tmpCurves.OrderByDescending(c => c.GetIndividualCurves().Max(i => i.StartPoint.Z))
                : tmpCurves.OrderByDescending(c => c.GetIndividualCurves().Max( i => i.StartPoint.X));
            intersectionCurves.Add(ordered.Last());

            if (tmpCurves.Count > 1)
                intersectionCurves.Add(ordered.First());

            var intersectOrigin = FindCenter(intersectionCurves[0].GetIndividualCurves(), selectedCase.HasYComponent);
            Point3D relOrigin;
            if (selectedCase.HillsidePlane == Plane.XY)
            {
                var pts = intersectionCurves[0].GetPointsByLength(intersectionCurves[0].Length() / 69.0);
                relOrigin = new Point3D(intersectOrigin.X, intersectOrigin.Y, pts.Min(v => v.Z));
            }
            else
                relOrigin = new Point3D(0, 0, intersectOrigin.Z);

            Center = FindCenter(cutRegion.ContourList.ToArray(), selectedCase.HasYComponent);

            /*
             *
             * Defining our reinforcement pad
             * --> has to be done relative to Neck and Shell as they can be all sorts of shapes
             *
             */

            var intersectSurf = new Region(intersectionCurves[1]).ConvertToSurface();
            var origin = FindCenter(intersectionCurves[1].GetIndividualCurves(), selectedCase.HasYComponent);
            intersectSurf.Regen(0.0001);

            intersectSurf.Project(origin, 0.0001, true, out var evalOrigin);
            normal = intersectSurf.Normal(evalOrigin);
            var normAngle = Math.Round(normal.AngleFromXY, 2);

            var flipNorm = !normAngle.Equals(0.0);

            if (normal.Z < 0 && Math.Round(normal.X, 3).Equals(0.0) && Math.Round(normal.Y, 3).Equals(0.0)
                || normal.X < 0 && Math.Round(normal.Z, 3).Equals(0.0) && Math.Round(normal.Y, 3).Equals(0.0))
                normal.Negate();

            var padFlushPlane = new Plane(origin, normal);
            Region padFace;
            Region weldFace;

            if (selectedCase.IsPadOffset)
            {
                var curves = uncutNeck.Section(padFlushPlane, 0.0001);
                curves = UtilityEx.GetConnectedCurves(curves, 0.0001);
                var curve = curves.OrderByDescending(c => c.StartPoint.DistanceTo(padFlushPlane.Origin)).First();

                padFace = curve.OffsetToRegion(selectedCase.RePadRadius, 0.0001, false);
                curve = padFace.ContourList.OrderByDescending(c => c.StartPoint.DistanceTo(padFlushPlane.Origin)).First();
                padFace = new Region(curve);

                weldFace = curve.OffsetToRegion(selectedCase.RePadRadius + selectedCase.WeldLength, 0.0001, false);
                curve = weldFace.ContourList.OrderByDescending(c => c.StartPoint.DistanceTo(padFlushPlane.Origin)).First();
                weldFace = new Region(curve);
            }
            else
            {
                padFlushPlane.Flip();
                padFace = new Region(new Circle(padFlushPlane, padFlushPlane.Origin, selectedCase.RePadRadius));
                weldFace = new Region(new Circle(padFlushPlane, padFlushPlane.Origin, selectedCase.RePadRadius + selectedCase.WeldLength));
            }

            var padExtrudeLength = extrudeLength + selectedCase.RePadThick;
            padFace.Translate(selectedCase.ExternalLength * normal);
            weldFace.Translate(selectedCase.ExternalLength * normal);

            Pad = padFace.ExtrudeAsBrep(padExtrudeLength);

            if (selectedCase.HillsidePlane == Plane.XY)
                selectedCase.RePadThick *= -1;
            var offSet = OffsetShell(intersectedSurfaces, selectedCase.RePadThick, flipNorm, false, relOrigin);
            var shellOuterSurf = OffsetShell(intersectedSurfaces, 0.0, !flipNorm, false, relOrigin);

            DifferenceShells(shellOuterSurf, ref Pad);
            DifferenceShells(offSet, ref Pad, false);

            Pad.ExtrudeRemove(cutRegion, extrudeLength);
            Shell.ExtrudeRemove(cutRegion, extrudeLength);
        }

        /*
         *
         * Various Helper methods
         *
         */

        public Brep OffsetShell(List<Surface> surfs, double thickness, bool flipNorm, bool inner, Point3D relOrigin)
        {
            var hasOffset = Math.Abs(thickness) > 0.0;

            var outerInner = GetOuterInnerSurfaces(surfs, relOrigin);
            var bound = (inner) ? outerInner.Item1 : outerInner.Item2;
            var curves = bound.Section(Plane.XZ, 0.0001).ToList();
            curves = UtilityEx.GetConnectedCurves(curves, 0.0001).ToList();
            var flat = curves.Any(c => c.IsInPlane(Plane.XY, 0.0001));
            var curve = (flat)
                ? curves.OrderByDescending(c => c.StartPoint.DistanceTo(Point3D.Origin)).Last()
                : curves.OrderByDescending(c => c.StartPoint.DistanceTo(Point3D.Origin)).First();
            var direction = 1.0;

            if (inner)
            {
                curve.Reverse();
                direction *= -1.0;
            }

            if ((curve.StartPoint.X > 0.0 && curve.EndPoint.X < 0.0) ||
                (curve.StartPoint.X < 0.0 && curve.EndPoint.X > 0.0))
            {
                var len = curve.Length();
                curve.SubCurve(curve.StartPoint, curve.GetPointsByLength(len / 2.0)[1], out curve);
            }

            var reg = new Region(curve);

            if (hasOffset && flat)
                reg.Translate(Vector3D.AxisZ * direction * thickness);
            else if (hasOffset)
                reg = new Region(curve.Offset(direction * thickness, Vector3D.AxisY, 0.0001, true));

            var b = reg.RevolveAsBrep(0.0, 2.0 * Math.PI, Vector3D.AxisZ, Point3D.Origin, 0.0001);

            if ((flipNorm ^ inner) && !flat)
                b.FlipNormal();

            return b;
        }

        public static bool DifferenceShells(Brep shell, ref Brep needsCut, bool wantOuter = true)
        {
            var pieces = new List<Brep>();
            pieces.AddRange(Brep.Difference(needsCut, shell) ?? new Brep[0]);
            if (!pieces.Any())
                return !Brep.Intersect(shell, needsCut, out var pts);
            needsCut = (wantOuter)
                ? pieces.OrderByDescending(p => p.Vertices.Max(v => v.MaximumCoordinate)).First()
                : pieces.OrderByDescending(p => p.Vertices.Max(v => v.MaximumCoordinate)).Last();
            return true;
        }

        private Point3D FindCenter(ICurve[] curves, bool hasYComponent = true)
        {
            var compCurve = new CompositeCurve(curves, 0.001, true);
            var lis = new List<Point3D>();
            var l = compCurve.Length();
            lis.AddRange(compCurve.GetPointsByLength(l / 69)); // nice.
            var x_mean = Math.Round(lis.Average(p => p.X), 3);
            var y_mean = (hasYComponent) ? Math.Round(lis.Average(p => p.Y), 3) : 0.0;
            var z_mean = Math.Round(lis.Average(p => p.Z), 3);
            return new Point3D(x_mean, y_mean, z_mean);
        }

        public static Tuple<Surface, Surface> GetOuterInnerSurfaces(List<Surface> surfs, Point3D origin)
        {
            Surface outer = null;
            Surface inner = null;
            foreach (var surf in surfs)
            {
                if (outer == null && inner == null)
                    outer = inner = surf;
                else
                {
                    outer.ClosestPointTo(origin, out var farthest);
                    inner.ClosestPointTo(origin, out var closest);
                    surf.ClosestPointTo(origin, out var candidate);
                    var candDist = origin.DistanceTo(candidate);
                    if (candDist > origin.DistanceTo(farthest))
                        outer = surf;
                    if (candDist < origin.DistanceTo(closest))
                        inner = surf;
                }
            }
            return new Tuple<Surface, Surface>(inner, outer);
        }

        private bool GetIntersectionCurves(Brep shell, Brep into,
            out List<ICurve> intersectionCurves, out List<Surface> intersectedSurfaces)
        {
            var neckSurfs = into.ConvertToSurfaces();
            var shellSurfs = shell.ConvertToSurfaces(0.0001);

            intersectionCurves = new List<ICurve>();
            intersectedSurfaces = new List<Surface>();
            foreach (var surf in shellSurfs)
            {
                var notAdded = true;
                foreach (var nSurf in neckSurfs)
                {
                    surf.IntersectWith(nSurf, 0.0001, out var foundCurves);
                    if (foundCurves != null && foundCurves.Length > 0)
                    {
                        intersectionCurves.AddRange(foundCurves);
                        if (notAdded)
                            intersectedSurfaces.Add(surf);
                        notAdded = false;
                    }
                }
            }

            return intersectionCurves.Count > 0;
        }
    }
}
