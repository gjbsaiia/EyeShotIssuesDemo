using System;
using System.Collections.Generic;
using devDept.Eyeshot.Entities;
using devDept.Geometry;

namespace SectioningProblem
{
    public enum Case
    {
        BaseCase,
        ConstantWidthCase,
        SphereCaseWBeta,
        SphereCase,
        Fillets,
    }
    public class CaseArgs
    {
        public Point3D ReferencePoint;
        public double ShellThickness;
        public double Theta;
        public double Beta;
        public double Hillside;
        public double ExternalLength;
        public double InternalLength;
        public double NeckRadius;
        public double NeckThickness;
        public double RePadThick;
        public double RePadRadius;
        public double WeldLength = 0.2;
        public bool IsPadOffset;
        public Brep Shell;
        public bool AddFillets;
        public Plane HillsidePlane;

        public bool HasYComponent => (!Hillside.Equals(0.0) && HillsidePlane != Plane.XY) ||
                                     (!Theta.Equals(0.0) && !(Hillside.Equals(0.0) && HillsidePlane == Plane.XY));
        public CaseArgs(Case which)
        {
            switch (which)
            {
                case Case.BaseCase:
                    BaseCase();
                    break;
                case Case.ConstantWidthCase:
                    BaseCase();
                    Beta = Math.PI/2.0 - 0.1;
                    IsPadOffset = true;
                    break;
                case Case.SphereCase:
                    SphereCase();
                    break;
                case Case.SphereCaseWBeta:
                    SphereCase();
                    Beta = 0.2;
                    Hillside = 4.0;
                    break;
                default:
                    throw new ArgumentException("Unknown case.");
            }
        }

        private void BaseCase()
        {
            var baseShape = new Circle(Point3D.Origin, 20.0);
            var ring = baseShape.OffsetToRegion(1.0, 0.0001, true);
            ShellThickness = 1.0;
            Shell = ring.ExtrudeAsBrep(60.0);
            ReferencePoint = new Point3D(20, 0, 30);
            Theta = 0.0;
            Beta = Math.PI/2.0;
            Hillside = 0.0;
            ExternalLength = 7.0;
            InternalLength = 2.0;
            NeckRadius = 1.0;
            NeckThickness = 0.2;
            RePadRadius = 3.0;
            RePadThick = 0.5;
            HillsidePlane = Plane.YZ;
            IsPadOffset = false;
            AddFillets = false;
        }

        private void SphereCase()
        {
            ShellThickness = 1.0;
            var shellRadius = 30.0;
            var p1 = new Point3D(shellRadius, 0,0);
            var p2 = new Point3D(0,0, shellRadius);
            var p3 = new Point3D(0,0, shellRadius + ShellThickness);
            var p4 = new Point3D( shellRadius + ShellThickness, 0,0);
            var center = new Point3D(0, 0,0);

            var curves = new List<ICurve>();
            curves.Add(new Arc(center, p1, p2));
            curves.Add(new Line(p2, p3));
            curves.Add(new Arc(center, p3, p4));
            curves.Add(new Line(p4, p1));
            UtilityEx.SortAndOrient(curves, true, true);

            var reg = new Region(curves);
            Shell = reg.RevolveAsBrep(2 * Math.PI, Vector3D.AxisZ, Point3D.Origin, 0.0001);
            ReferencePoint = new Point3D(0, 0, 30);
            Theta = 0.0;
            Beta = 0.0;
            Hillside = 0.0;
            ExternalLength = 7.0;
            InternalLength = 2.0;
            NeckRadius = 2.0;
            NeckThickness = 0.5;
            RePadRadius = 5.0;
            RePadThick = 0.5;
            HillsidePlane = Plane.XY;
            IsPadOffset = false;
            AddFillets = false;
        }

    }
}
