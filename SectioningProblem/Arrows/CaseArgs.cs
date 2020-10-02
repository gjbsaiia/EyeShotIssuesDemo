using System;
using devDept.Eyeshot.Entities;
using devDept.Geometry;

namespace SectioningProblem
{
    public enum Case
    {
        BaseCase,
        ConstantWidthCase,
        SphereCase,
        Fillets,
        SolidFillets
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
        public bool IsPadOffset;
        public Brep Shell;
        public bool CanClean;
        public bool AddFillets;
        public bool MakeSolidFillets;
        public Plane HillsidePlane;
        public CaseArgs(Case which)
        {
            switch (which)
            {
                case Case.BaseCase:
                    BaseCase();
                    break;
                case Case.ConstantWidthCase:
                    BaseCase();
                    Beta = Math.PI / 4.0;
                    IsPadOffset = true;
                    CanClean = true;
                    break;
                case Case.SphereCase:
                    SphereCase();
                    break;
                case Case.Fillets:
                    BaseCase();
                    CanClean = true;
                    AddFillets = true;
                    MakeSolidFillets = false;
                    break;
                case Case.SolidFillets:
                    BaseCase();
                    CanClean = true;
                    AddFillets = true;
                    MakeSolidFillets = true;
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
            Beta = 0.0;
            Hillside = 0.0;
            ExternalLength = 7.0;
            InternalLength = 2.0;
            NeckRadius = 1.0;
            NeckThickness = 0.2;
            RePadRadius = 3.0;
            RePadThick = 0.5;
            HillsidePlane = Plane.YZ;
            IsPadOffset = false;
            CanClean = false;
            AddFillets = false;
            MakeSolidFillets = false;
        }

        private void SphereCase()
        {
            var arc = new Arc(Plane.XZ, Point3D.Origin, 30.0, 0.0, Math.PI/2.0);
            var reg = arc.OffsetToRegion(1.0, 0.0001, true);
            ShellThickness = 1.0;
            Shell = reg.RevolveAsBrep(2*Math.PI, Vector3D.AxisZ, Point3D.Origin, 0.0001);
            ReferencePoint = new Point3D(0, 0, 30);
            Theta = 0.0;
            Beta = Math.PI / 2.0;
            Hillside = 0.0;
            ExternalLength = 7.0;
            InternalLength = 2.0;
            NeckRadius = 2.0;
            NeckThickness = 0.5;
            RePadRadius = 5.0;
            RePadThick = 0.5;
            HillsidePlane = Plane.XY;
            IsPadOffset = false;
            CanClean = false;
            AddFillets = false;
            MakeSolidFillets = false;
        }

    }
}
