using System;
using System.Collections.Generic;
using System.Linq;
using devDept.Eyeshot.Entities;
using devDept.Geometry;

namespace SectioningProblem
{
    public class Nozzle
    {
        public Brep Shell;
        public Brep Neck;
        public Brep Pad;
        public List<Brep> Welds;
        public bool CanClean;

        public Nozzle(CaseArgs args)
        {
            RunCase(args);
        }

        public void RunCase(CaseArgs selectedCase)
        {

            // shell that we're attaching the neck too
            Shell = selectedCase.Shell;
            // indicates whether or not we're going to allow cleaning of models
            CanClean = selectedCase.CanClean;

            // length that we'll extrude our neck
            var extrudeLength = selectedCase.ExternalLength + selectedCase.InternalLength + selectedCase.ShellThickness;


            // normal to define our plane with various angles (all cases I defined are orthogonal)
            var normal = new Vector3D(Math.Cos(selectedCase.Theta) * Math.Cos(selectedCase.Beta),
                Math.Sin(selectedCase.Theta) * Math.Cos(selectedCase.Beta), Math.Sin(selectedCase.Beta));
            normal.Normalize();

            // plane 'flush' to surface of cylinder
            var flushPlane = new Plane(selectedCase.ReferencePoint, normal);

            // plane we're going to extrude from
            var extrudePlane = flushPlane.Clone() as Plane;
            extrudePlane.Translate(normal * selectedCase.ExternalLength);
            extrudePlane.Flip();

            // Vector to define a case.Hillside offset
            Vector3D hillSideVector = (selectedCase.HillsidePlane == Plane.YZ)
                ? flushPlane.AxisY * selectedCase.Hillside : flushPlane.AxisX * selectedCase.Hillside;

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
            if (CanClean && Neck.FixTopology(out var fixedNeck)) // feel like this shouldn't be necessary
                Neck = fixedNeck;

            /*
             *
             * Defining our reinforcement pad
             * --> has to be done relative to Neck and Shell as they can be all sorts of shapes
             *
             */

            // find outer neck surface
            var neckSurfs = Neck.ConvertToSurfaces().Where(s => !(s is PlanarSurface)).ToList();
            var neckOuterInner = GetOuterInnerSurfaces(neckSurfs, Neck.Vertices.Max(v => v.MaximumCoordinate));
            var neckOuterSurf = neckOuterInner.Item1;

            // get the shell surfaces where it is possible to have a neck attached
            var shellSurfs = GetSignificantSurfaces(Shell, selectedCase.HillsidePlane);
            var allIntersectCurves = new List<ICurve>();
            foreach (var surf in shellSurfs)
            {
                surf.IntersectWith(neckOuterSurf, 0.0001, out var foundCurves);
                if (foundCurves != null)
                    allIntersectCurves.AddRange(foundCurves);
            }

            var intersectCurves = UtilityEx.GetConnectedCurves(allIntersectCurves, 0.0001); // thank you for these, btw
            var outerIntersect =
                intersectCurves.OrderByDescending(c => c.StartPoint.DistanceTo(Point3D.Origin)).First();

            // Finds the 'center' of our intersection between neck and shell (only has to be perfect in the Z and Y components)
            var origin = FindCenter(outerIntersect.GetIndividualCurves());
            // normal to surface of the cylinder
            double normalAngle = (selectedCase.HillsidePlane == Plane.XY) ? Math.PI / 2.0 : 0.0;

            // reinforcement pad should always be orthogonal to the shell surface
            var padNorm = new Vector3D(Math.Cos(selectedCase.Theta) * Math.Cos(normalAngle), Math.Sin(selectedCase.Theta) * Math.Cos(normalAngle), Math.Sin(normalAngle));
            var padFlush = new Plane(origin, padNorm);
            var padExtrude = padFlush.Clone() as Plane;
            var dist = padFlush.Origin.DistanceTo(Point3D.Origin);
            padExtrude.Translate(selectedCase.ExternalLength * padNorm);
            padExtrude.Flip();
            Region padFace;
            if(!selectedCase.IsPadOffset) // case where we want a reinforcement pad with constant diameter
                padFace = new Region(new Circle(padFlush, padFlush.Origin, selectedCase.RePadRadius));
            else // case where we want a reinforcement pad with constant width
            {
                var neckCp = Neck.Clone() as Brep;
                var curves = neckCp.Section(flushPlane, 0.0001);
                var curve = curves.OrderByDescending(c => (c.StartPoint.Z > c.EndPoint.Z) ? c.StartPoint.Z : c.EndPoint.Z).First();
                var neckNorm = padNorm.Clone() as Vector3D;
                neckNorm.Reverse();
                padFace = new Region(curve.Offset(selectedCase.RePadRadius, neckNorm, 0.0001, true));
                padFace.FlipNormal();
            }
            padFace.Translate(selectedCase.ExternalLength * padNorm);
            padFace.FlipNormal();
            Pad = padFace.ExtrudeAsBrep(selectedCase.ExternalLength + dist);
            if (CanClean && Pad.FixTopology(out var fixedBrep)) // feel like this shouldn't be necessary
                Pad = fixedBrep;

            // make a new Brep that is shifted outward by the pad thickness we want
            var offSet = OffsetShell(shellSurfs, selectedCase.RePadThick);
            if (CanClean && offSet.FixTopology(out var fixedOffset)) // feel like this shouldn't be necessary
                offSet = fixedOffset;

            // make our outer cut to the pad
            DifferenceShells(offSet, ref Pad, false);

            Welds = new List<Brep>();
            if (selectedCase.AddFillets) // this doesn't work. Not sure how to achieve the welds I want as solids and not just surfaces
            {
                Func<Brep, Brep, Plane, double, bool, List<Brep>> solidOrSurf; 
                // just determines whether we make the fillets as a surface or attempt to make it solid
                if(selectedCase.MakeSolidFillets)
                    solidOrSurf = MakeWeld;
                else
                    solidOrSurf = MakeWelds;
                Welds.AddRange(solidOrSurf(Pad, Neck, selectedCase.HillsidePlane, selectedCase.RePadThick * 0.7, false));
                Welds.AddRange(solidOrSurf(Shell, Pad, selectedCase.HillsidePlane, selectedCase.RePadThick * 0.7, false));
                Welds.AddRange(solidOrSurf(Shell, Pad, selectedCase.HillsidePlane, selectedCase.RePadThick * 0.7, true));
            }

            // make our inner cut to the pad
            DifferenceShells(Shell, ref Pad);

            // cut hole through shell for neck
            Shell.ExtrudeRemove(cutRegion, extrudeLength);
            // cut hole through pad for neck
            Pad.ExtrudeRemove(cutRegion, extrudeLength);

        }

        /*
         *
         * Various Helper methods
         *
         */

        public Brep OffsetShell(List<Surface> surfs, double thickness)
        {
            var norm = Vector3D.AxisY;

            var bound = GetOuterInnerSurfaces(surfs).Item2;
            var curves = bound.Section(Plane.XZ, 0.0001).ToList();

            curves = UtilityEx.GetConnectedCurves(curves, 0.0001).ToList();
            var curve = curves[UtilityEx.GetOuterIndex(curves, 0.001)];

            if (curve.StartPoint.X >= 0.0 || curve.EndPoint.X >= 0.0)
            {
                norm.Reverse();
            }
            curve = curve.Offset(thickness, norm, 0.0001, true);

            var brep = curve.RevolveAsBrep(0.0, Math.PI * 2, Vector3D.AxisZ, Point3D.Origin, 0.0001);
            brep.FlipNormal();
            return brep;
        }

        public bool DifferenceShells(Brep shell, ref Brep needsCut, bool wantOuter = true)
        {
            var pieces = new List<Brep>();
            pieces.AddRange(Brep.Difference(needsCut, shell) ?? new Brep[0]);
            if (!pieces.Any())
                return !Brep.Intersect(shell, needsCut, out var pts);
            if (wantOuter)
                needsCut = pieces.OrderByDescending(p => p.Vertices.Max(v => v.MaximumCoordinate)).First();
            else
                needsCut = pieces.OrderBy(p => p.Vertices.Max(v => v.MaximumCoordinate)).First();
            return true;
        }

        public static Point3D FindCenter(ICurve[] curves)
        {
            var compCurve = new CompositeCurve(curves, 0.001, true);
            var lis = new List<Point3D>();
            var l = compCurve.Length();
            lis.AddRange(compCurve.GetPointsByLength(l / 69)); // nice.
            var x_mean = lis.Average(p => p.X);
            var y_mean = lis.Average(p => p.Y);
            var z_mean = lis.Average(p => p.Z);
            return new Point3D(x_mean, y_mean, z_mean);
        }

        public List<Surface> GetSignificantSurfaces(Brep shell, Plane hillSidePlane)
        {
            var shellSurfs = shell.ConvertToSurfaces().ToList();
            if (shellSurfs.Any(s => !(s is CylindricalSurface) && (s is RevolvedSurface)))
                shellSurfs = shellSurfs.Where(s => !(s is PlanarSurface)).ToList();
            else
            {
                shellSurfs.ForEach(s => s.Regen(0.0001));
                if (hillSidePlane != Plane.XY)
                {
                    var maxBound = shellSurfs.Max(s => s.BoxMax.Z - s.BoxMin.Z);
                    shellSurfs = shellSurfs.Where(s => (s.BoxMax.Z - s.BoxMin.Z) == maxBound).ToList();
                }
                else
                {
                    var maxBound = shellSurfs.Max(s => s.BoxMax.DistanceTo(s.BoxMin));
                    shellSurfs = shellSurfs.Where(s => s.BoxMax.DistanceTo(s.BoxMin) == maxBound).ToList();
                }
            }
            return shellSurfs;
        }

        public Tuple<Surface, Surface> GetOuterInnerSurfaces(List<Surface> surfs, double zoff = 0.0)
        {
            var origin = new Point3D(0, 0, zoff);
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

        public List<Brep> MakeWelds(Brep shell, Brep neck, Plane hillSidePlane, double sz, bool inner = false)
        {
            var shellList = new List<Surface>();
            var neckList = new List<Surface>();
            var shellSurfaces = GetSignificantSurfaces(shell, hillSidePlane);
            var shellOuterInner = GetOuterInnerSurfaces(shellSurfaces);
            var shellSurf = (inner) ? shellOuterInner.Item1 : shellOuterInner.Item2;
            shellList.Add(shellSurf);
            var zoff = shell.Vertices.Max(v => v.Z);
            var neckSurfaces = GetSignificantSurfaces(neck, Plane.ZX);
            var neckInnerOuter = GetOuterInnerSurfaces(neckSurfaces, zoff);
            var neckOuter = neckInnerOuter.Item1;
            neckList.Add(neckOuter);
            Surface.Chamfer(shellList, neckList, sz, 0.0001, false,
                false, false, false, false, false, out var fillets, out var shellLeft, out var neckLeft);

            var lis = new List<Brep>();
            fillets.ToList().ForEach(s => lis.Add(s.ConvertToBrep()));
            return lis;
        }

        public List<Brep> MakeWeld(Brep shell, Brep neck, Plane hillSidePlane, double sz, bool inner = false)
        {
            var shellList = new List<Surface>();
            var neckList = new List<Surface>();
            var shellSurfaces = GetSignificantSurfaces(shell, hillSidePlane);
            var shellOuterInner = GetOuterInnerSurfaces(shellSurfaces);
            var shellSurf = (inner) ? shellOuterInner.Item1 : shellOuterInner.Item2;
            shellList.Add(shellSurf);
            var zoff = shell.Vertices.Max(v => v.Z);
            var neckSurfaces = GetSignificantSurfaces(neck, Plane.ZX);
            var neckInnerOuter = GetOuterInnerSurfaces(neckSurfaces, zoff);
            var neckOuter = neckInnerOuter.Item1;
            neckList.Add(neckOuter);
            Surface.Chamfer(shellList, neckList, sz, 0.1, false,
                false, true, true, false, false, out var fillets, out var shellLeft, out var neckLeft);

            var filletCurves = new List<ICurve>();

            fillets.ToList().ForEach(f => filletCurves.AddRange(f.ExtractLoops3D()));
            shellLeft.ToList().ForEach(s => filletCurves.AddRange(s.ExtractLoops3D()));
            neckLeft.ToList().ForEach(n => filletCurves.AddRange(n.ExtractLoops3D()));

            var weldCurves = new List<ICurve>();
            foreach (var c in filletCurves)
            {
                if (c is CompositeCurve)
                    weldCurves.AddRange(c.GetIndividualCurves());
                else
                    weldCurves.Add(c);
            }

            weldCurves = UtilityEx.GetConnectedCurves(weldCurves, 0.1).ToList();

            if (weldCurves.Any(c => !c.IsClosed))
                return new List<Brep>();

            var breps = new List<Brep>();
            breps.Add( Brep.Loft(weldCurves.ToArray()));

            return breps;
        }
    }
}
