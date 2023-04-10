// Program saxis currently just displays (port :8080) a six axis robot
// following a Hilbert path in 4 different path modes: joint
// interpolated; arc'd edges; linear interpolated; linear but cutting
// corners; repeat.
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"net/http"
	"sync"
	"time"

	"zappem.net/pub/kinematics/sixaxis"
	"zappem.net/pub/math/geom"
	"zappem.net/pub/math/hilbert"
)

var (
	addr   = flag.String("addr", ":8080", "address to serve from")
	dir    = flag.String("dir", "./html", "directory of content")
	detail = flag.Int("detail", 2, "depth of hilbert curve")
	dt     = flag.Float64("dt", .25, "seconds per step")
	arc    = flag.Float64("arc", 120, "arc angle in degrees")
	width  = flag.Float64("width", 3, "edge width of hilbert surface")
	debug  = flag.Bool("debug", false, "be more verbose")

	stepTime time.Duration
)

type Joint struct {
	Width, Length float64
	Min, Max      float64
	Axis          string
}

// view holds the webserver state.

type view struct {
	done    chan int
	quitter chan struct{}
	Hilbert []geom.Vector
	Joints  []Joint
	mu      sync.RWMutex
	robot   *sixaxis.Robot
	pcount  int
	program [][]sixaxis.Pace
}

func (v *view) serve(w http.ResponseWriter, r *http.Request) {
	url := r.URL.Path
	switch url {
	default:
		http.ServeFile(w, r, "./html/"+url)
	}
}

type Query struct {
	Cmd    string
	Pcount int
}

type Response struct {
	Pose    sixaxis.Pose
	Pcount  int
	Program [][]sixaxis.Pace
}

type Scene struct {
	Hilbert []geom.Vector
	Robot   []Joint
	Pose    sixaxis.Pose
}

func (v *view) rpc(w http.ResponseWriter, r *http.Request) {
	rpc := r.FormValue("rpc")
	var q Query
	if err := json.Unmarshal([]byte(rpc), &q); err != nil {
		http.Error(w, "invalid rpc: "+err.Error(), http.StatusBadRequest)
		return
	}
	var j []byte
	var err error
	v.mu.RLock()
	defer v.mu.RUnlock()
	if v.pcount == q.Pcount {
		v.done <- q.Pcount
	}
	switch q.Cmd {
	case "scene":
		s := Scene{
			Hilbert: v.Hilbert,
			Robot:   v.Joints,
			Pose:    v.robot.Pose(),
		}
		j, err = json.Marshal(s)
	case "hilbert":
		j, err = json.Marshal(v.Hilbert)
	case "status":
		p := Response{
			Pose:   v.robot.Pose(),
			Pcount: v.pcount,
		}
		if q.Pcount < v.pcount {
			p.Program = v.program
		}
		j, err = json.Marshal(p)
	default:
		http.Error(w, "unrecognized command", http.StatusBadRequest)
		return
	}
	if err != nil {
		panic("no json?")
	}
	w.Header().Set("Content-Type", "application/json")
	w.Write(j)
}

// twoD generates a vector trace for a hilbert curve mapped out in the
// z-x plane of a basis, m, from point r and covering a square of edge
// side length. Our convention is x-y is the horizontal plane and up
// is the positive z direction.
func TwoD(n uint, m geom.Matrix, r geom.Vector, edge float64) (pts []geom.Vector) {
	w := uint(1) << (n + 1)
	w2 := w * w
	f := 1.0 / (float64(w) - 1)
	dx, dy := m.Z().Scale(edge*f), m.X().Scale(edge*f)
	for i := uint(0); i < w2; i++ {
		x, y, err := hilbert.StepXY(n, i)
		if err != nil {
			panic(fmt.Sprintf("hilbert.StepXY failed: %v", err))
		}
		pts = append(pts, r.AddS(dx, float64(x)).AddS(dy, float64(y)))
	}
	return
}

// Force the pose of the robot from a list of joint angles.
func (v *view) updatePose(js []geom.Angle) {
	v.mu.Lock()
	for k, a := range js {
		v.robot.SetJ(k, a)
	}
	v.mu.Unlock()
}

func (v *view) stepping() {
	m := geom.RX(geom.Degrees(90))
	oldPt := v.Hilbert[0]

	v.mu.RLock()
	js := v.robot.Inverse(m, oldPt)
	best, err := v.robot.Closest(nil, js)
	v.mu.RUnlock()
	if err != nil {
		panic("initial pose is bogus: " + err.Error())
	}
	v.updatePose(js[best])

	// Precompute all of the poses.
	oldPose := v.robot.Pose()
	var poses []sixaxis.Pose
	for _, pt := range v.Hilbert {
		aPt := pt.Add(geom.Y(.01))
		v.mu.RLock()
		js := v.robot.Inverse(m, aPt)
		best, err := v.robot.Closest(&oldPose, js)
		v.mu.RUnlock()
		if err != nil {
			panic(err)
		}
		pose := sixaxis.Pose{
			V: aPt,
			M: m,
			J: js[best],
		}
		poses = append(poses, pose)
		oldPose = pose
	}

	for flav := 0; ; flav = (flav + 1) % 4 {
		var program [][]sixaxis.Pace

		for i, pose := range poses {
			var paces <-chan sixaxis.Pace
			tFlav := flav
			if i == 0 || i == len(poses)-1 {
				tFlav = 2
			}
			switch tFlav {
			case 0:
				paces, err = v.robot.Joined(nil, *dt, pose, v.quitter)
			case 1:
				axis := geom.Y(1)
				paces, err = v.robot.Arc(nil, *dt, pose, axis, geom.Degrees(*arc), v.quitter)
			case 2:
				paces, err = v.robot.Linear(nil, *dt, pose, v.quitter)
			case 3:
				paces, err = v.robot.Near(nil, *dt, poses[i-1], pose, poses[i+1], *width/7, *width/7, v.quitter)
			}
			if err != nil {
				panic(err)
			}
			var ps []sixaxis.Pace
			for pace := range paces {
				if pace.Frac <= 0 {
					panic(fmt.Sprintf("flavor=%d step=%d failed", flav, i))
				}
				ps = append(ps, pace)
				if *debug {
					fmt.Printf("[%d,%d] H pace = %v\n", v.pcount, i, pace)
				}
				v.updatePose(pace.J)
			}
			program = append(program, ps)
			oldPose = pose
		}

		ptN := oldPose.V.Add(geom.Y(.2))
		js = v.robot.Inverse(m, ptN)
		best, err = v.robot.Closest(&oldPose, js)
		if err != nil {
			panic(err)
		}
		pose := sixaxis.Pose{
			V: ptN,
			M: m,
			J: js[best],
		}
		paces, err := v.robot.Linear(&oldPose, *dt, pose, v.quitter)
		if err != nil {
			panic(err)
		}
		var ps []sixaxis.Pace
		for pace := range paces {
			ps = append(ps, pace)
			if *debug {
				fmt.Printf("[%d] pace = %v\n", v.pcount, pace)
			}
			v.updatePose(pace.J)
		}
		program = append(program, ps)

		pt0 := v.Hilbert[0].Add(geom.Y(.2))
		js = v.robot.Inverse(m, pt0)
		best, err = v.robot.Closest(&poses[0], js)
		if err != nil {
			panic(err)
		}
		pose2 := sixaxis.Pose{
			V: pt0,
			M: m,
			J: js[best],
		}
		paces, err = v.robot.Linear(&pose, 2**dt, pose2, v.quitter)
		if err != nil {
			panic(err)
		}
		ps = nil
		for pace := range paces {
			ps = append(ps, pace)
			if *debug {
				fmt.Printf("[%d] pace = %v\n", v.pcount, pace)
			}
			v.updatePose(pace.J)
		}

		v.mu.Lock()
		v.program = append(program, ps)
		v.pcount++
		pcount := v.pcount
		v.mu.Unlock()

		oldPose = v.robot.Pose()

		for n := range v.done {
			if n == pcount {
				break
			}
		}
	}
}

func newView(quitter chan struct{}) {
	v := &view{
		done:    make(chan int, 3),
		quitter: quitter,
		Joints: []Joint{
			{Min: -170, Max: 170, Axis: "z", Length: 1, Width: 1.5},
			{Min: -120, Max: 120, Axis: "x", Length: 2, Width: .8},
			{Min: -120, Max: 120, Axis: "x", Length: 1.5, Width: .7},
			{Min: -170, Max: 170, Axis: "z", Length: 1, Width: .6},
			{Min: -120, Max: 120, Axis: "x", Length: .5, Width: .4},
			{Min: -360, Max: 360, Axis: "z", Length: 0, Width: .3},
		},
	}
	var err error
	var js []sixaxis.Joint
	for _, j := range v.Joints {
		x := sixaxis.Joint{
			Min:    geom.Degrees(j.Min),
			Max:    geom.Degrees(j.Max),
			Offset: j.Length,
		}
		switch j.Axis {
		case "x":
			x.Rot = geom.RX
		case "y":
			x.Rot = geom.RY
		case "z":
			x.Rot = geom.RZ
		}
		js = append(js, x)
	}
	v.robot, err = sixaxis.NewRobot(js...)
	if err != nil {
		panic(fmt.Sprintf("unable to define robot: %v", err))
	}
	v.robot.Precision = geom.Radians(0.001 * math.Pi)

	v.Hilbert = TwoD(uint(*detail), geom.I, geom.V(-*width/2, -3.8, -(*width-2)/2), *width)

	go v.stepping()

	// Simple webserver:
	http.HandleFunc("/", v.serve)
	http.HandleFunc("/rpc", v.rpc)
	http.ListenAndServe(*addr, nil)
}

func main() {
	flag.Parse()

	stepTime = time.Duration(*dt) * time.Second

	q := make(chan struct{})
	go newView(q)

	// wait for the end.
	<-q
}
