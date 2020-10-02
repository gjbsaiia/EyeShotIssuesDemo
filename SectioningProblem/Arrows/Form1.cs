using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;
using devDept.Eyeshot.Entities;
using devDept.Eyeshot;
using devDept.Geometry;

namespace SectioningProblem
{
    public partial class Form1 : Form
    {
        private Nozzle _nozzle;
        private bool _firstRun;
        private Color _caseColor = Color.LightBlue;

        public Form1()
        {
            InitializeComponent();
            _firstRun = true;
            //model1.Unlock(""); // For more details see 'Product Activation' topic in the documentation.

        }

        protected override void OnLoad(EventArgs e)
        {
            // adjusts grid extents ad step
            model1.ActiveViewport.Grid.Max = new Point3D(200, 200);
            model1.ActiveViewport.Grid.Step = 25;
            model1.ActiveViewport.OriginSymbol.Visible = false;

            slice.BringToFront();
            slice.BackColor = Color.DarkSalmon;

            textBox.BringToFront();

            foreach (var box in cases)
            {
                box.BringToFront();
            }
            ResetColor(cases[0].Name);

            _nozzle = new Nozzle(new CaseArgs(Case.BaseCase));

            AddObjects();

            // sets trimetric view
            model1.SetView(viewType.Trimetric);

            // fits the model in the viewport
            model1.ZoomFit();

            base.OnLoad(e);
        }

        private void ResetColor(string caseName)
        {
            foreach (var box in cases)
            {
                if (box.Name == caseName)
                {
                    box.CheckState = CheckState.Checked;
                    box.BackColor = Color.FromArgb(60, _caseColor);
                }
                else
                {
                    box.CheckState = CheckState.Unchecked;
                    box.BackColor = _caseColor;
                }
            }
        }

        private void AddObjects()
        {
            /*
             * Clear out any previous models
             */

            if (!_firstRun)
            {
                model1.Layers.Clear(false);
            }
            else
                _firstRun = false;

            model1.Layers.Add("Whole");
            model1.Layers.Add("Sliced");

            /*
             * Add our objects to the drawing
             */

            var colors = new Color[] { Color.DarkSlateBlue, Color.CornflowerBlue, Color.CadetBlue, Color.Goldenrod };

            var objs = new List<Brep>();
            var nonWelds = new Brep[] {_nozzle.Shell, _nozzle.Neck, _nozzle.Pad};
            objs.AddRange(nonWelds);
            objs.AddRange(_nozzle.Welds);
            // objects sliced along XZ Plane

            var cutObjs = new List<Brep>();
            
            for (int i = 0; i < objs.Count; i++)
            {
                var cut = (objs[i].Clone() as Brep);
                cut.CutBy(Plane.XZ);
                cutObjs.Add(cut);
            }

            int j = 0;
            while (j < nonWelds.Length)
            {
                model1.Entities.Add(objs[j], "Whole", colors[j]);
                model1.Layers["Whole"].Visible = true;
                model1.Entities.Add(cutObjs[j], "Sliced", colors[j]);
                model1.Layers["Sliced"].Visible = true;
                j++;
            }

            int save = j;
            while (j < objs.Count)
            {
                model1.Entities.Add(objs[j], "Whole", colors[save]);
                model1.Layers["Whole"].Visible = true;
                model1.Entities.Add(cutObjs[j], "Sliced", colors[save]);
                model1.Layers["Sliced"].Visible = true;
                j++;
            }
        }



        private void _sliceCylinder(Object o, EventArgs args)
        {
            if (model1.Layers["Whole"].Visible)
            {
                model1.Layers["Whole"].Visible = false;
                model1.Layers["Sliced"].Visible = true;
                slice.Text = "Whole";
                model1.SetView(viewType.Front);
            }
            else
            {
                model1.Layers["Whole"].Visible = true;
                model1.Layers["Sliced"].Visible = false;
                slice.Text = "Cut";
                model1.SetView(viewType.Trimetric);
            }
            slice.Update();
            model1.Entities.Regen();
            model1.Invalidate();
        }

        private void _selectedCase(Object o, EventArgs args)
        {
            var button = o as CheckBox;
            if (button != null && button.Tag is Case selectedCase)
            {
                textBox.Visible = false;
                ResetColor(button.Name);
                model1.Layers["Whole"].Visible = false;
                model1.Layers["Sliced"].Visible = false;
                model1.Entities.Regen();
                model1.Invalidate();
                model1.Refresh();

                try
                {
                    _nozzle.RunCase(new CaseArgs(selectedCase));
                    AddObjects();
                }
                catch(Exception e)
                {
                    textBox.Multiline = true;
                    textBox.ScrollBars = ScrollBars.Vertical;
                    textBox.TextAlign = HorizontalAlignment.Left;
                    textBox.Text = "Exception " + e.ToString() + "thrown at " +
                                   e.TargetSite.ToString() + ": " + e.Message + ". ";
                    textBox.Visible = true;
                }

                model1.Entities.Regen();
                model1.Invalidate();

            }
        }
                
    }

}