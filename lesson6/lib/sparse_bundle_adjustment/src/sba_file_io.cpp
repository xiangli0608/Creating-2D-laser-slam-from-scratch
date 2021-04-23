#include "sparse_bundle_adjustment/sba_file_io.h"
#include <map>

using namespace sba;
using namespace Eigen;
using namespace frame_common;
using namespace std;

int sba::readBundlerFile(const char *filename, SysSBA& sbaout)
{ 
    // Create vectors to hold the data from the bundler file. 
    vector< Vector3d, Eigen::aligned_allocator<Vector3d> > camps;	// cam params <f d1 d2>
    vector< Matrix3d, Eigen::aligned_allocator<Matrix3d> > camRs;	// cam rotation matrix
    vector< Vector3d, Eigen::aligned_allocator<Vector3d> > camts;	// cam translation
    vector< Vector3d, Eigen::aligned_allocator<Vector3d> > ptps;	// point position
    vector< Vector3i, Eigen::aligned_allocator<Vector3i> > ptcs;	// point color
    vector< vector< Vector4d, Eigen::aligned_allocator<Vector4d> > > ptts; // point tracks - each vector is <camera_index kp_idex u v>

    int ret = ParseBundlerFile(filename, camps, camRs, camts, ptps, ptcs, ptts);
    if (ret < 0)
        return -1;
        
    int ncams = camps.size();
    int npts  = ptps.size();
    int nprjs = 0;
    for (int i=0; i<npts; i++)
        nprjs += (int)ptts[i].size();
    /* cout << "Points: " << npts << "  Tracks: " << ptts.size() 
         << "  Projections: " << nprjs << endl; */
         
    cout << "Setting up nodes..." << flush;
    for (int i=0; i<ncams; i++)
    {
        // camera params
        Vector3d &camp = camps[i];
        CamParams cpars = {camp[0],camp[0],0,0,0}; // set focal length, no offset

        //
        // NOTE: Bundler camera coords are rotated 180 deg around the X axis of
        //  the camera, so Z points opposite the camera viewing ray (OpenGL).
        // Note quite sure, but I think this gives the camera pose as
        //  [-R' -R'*t]

        // rotation matrix
        Matrix3d m180x;		// rotate 180 deg around X axis, to convert Bundler frames to SBA frames
        m180x << 1, 0, 0, 0, -1, 0, 0, 0, -1;
        Matrix3d camR = m180x * camRs[i]; // rotation matrix
        Quaternion<double> frq(camR.transpose());	// camera frame rotation, from Bundler docs
        if (frq.w() < 0.0)	// w negative, change to positive
        {
            frq.x() = -frq.x();
            frq.y() = -frq.y();
            frq.z() = -frq.z();
            frq.w() = -frq.w();
        }
        
        frq.normalize();

        // translation
        Vector3d &camt = camts[i];
        Vector4d frt;
        frt.head<3>() = -camRs[i].transpose() * camt; // camera frame translation, from Bundler docs
        frt[3] = 1.0;

        Node nd;
        
        sbaout.addNode(frt, frq, cpars);
    }
    // cout << "done" << endl;

    // set up points
    cout << "Setting up points..." << flush;
    for (int i=0; i<npts; i++)
    {
        // point
        Vector3d &ptp = ptps[i];
        Point pt;
        pt.head<3>() = ptp;
        pt[3] = 1.0;
        sbaout.addPoint(pt);
    }
    // cout << "done" << endl;


    sbaout.useLocalAngles = true;    // use local angles
    sbaout.nFixed = 1;

    // set up projections
    int ntot = 0;
    cout << "Setting up projections..." << flush;
    for (int i=0; i<npts; i++)
    {
      // track
      vector<Vector4d, Eigen::aligned_allocator<Vector4d> > &ptt = ptts[i];
      int nprjs = ptt.size();
      for (int j=0; j<nprjs; j++)
        {
	  // projection
	  Vector4d &prj = ptt[j];
	  int cami = (int)prj[0];
	  Vector2d pt = prj.segment<2>(2);
	  pt[1] = -pt[1];	// NOTE: Bundler image Y is reversed
	  if (cami >= ncams)
	    cout << "*** Cam index exceeds bounds: " << cami << endl;
	  sbaout.addMonoProj(cami,i,pt); // Monocular projections
	  ntot++;
        }
    }
    cout << "done" << endl;
    
    return 0;
}

int sba::writeBundlerFile(const char *filename, SysSBA& sbain)
{
    ofstream outfile(filename, ios_base::trunc);
    if (!outfile)
    {
        cout << "Can't open file " << filename << endl;
        return -1;
    }
    
    outfile.precision(10);
    outfile.setf(ios_base::scientific);
    
    unsigned int i = 0;
    
    outfile << "# Bundle file v0.3" << endl;
    // First line is number of cameras and number of points.
    outfile << sbain.nodes.size() << ' ' << sbain.tracks.size() << endl;
    
    // Set up transform matrix for camera parameters
    Matrix3d m180x;		// rotate 180 deg around X axis, to convert Bundler frames to SBA frames
    m180x << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    
    
    // Then goes information about each camera, in <f> <k1> <k2>\n<R>\n<t> format.
    for (i = 0; i < sbain.nodes.size(); i++)
    {
        // Assuming fx = fy and using fx. Don't use k1 or k2.
        outfile << sbain.nodes[i].Kcam(0, 0) << ' ' << 0.0 << ' ' << 0.0 << endl;
        
        Quaternion<double> quat(sbain.nodes[i].qrot);
        /* cout << "\nQuat: [ " << sbain.nodes[i].qrot << " ]\n"; */ 
        quat.normalize();
        Matrix3d rotmat = m180x * quat.toRotationMatrix().transpose();
                       
        outfile << rotmat(0, 0) << ' ' << rotmat(0, 1) << ' ' << rotmat(0, 2) << endl;
        outfile << rotmat(1, 0) << ' ' << rotmat(1, 1) << ' ' << rotmat(1, 2) << endl;
        outfile << rotmat(2, 0) << ' ' << rotmat(2, 1) << ' ' << rotmat(2, 2) << endl;
        
        Vector3d trans = sbain.nodes[i].trans.head<3>();
        trans = -rotmat*trans;
        outfile << trans(0) << ' ' << trans(1) << ' ' << trans(2) << endl; 
    }
    
    outfile.setf(ios_base::fixed);
    
    // Then goes information about each point. <pos>\n<color>\n<viewlist>
    for (i = 0; i < sbain.tracks.size(); i++)
    {
        // World <x y z>
        outfile << sbain.tracks[i].point(0) << ' ' << sbain.tracks[i].point(1) 
                << ' ' << sbain.tracks[i].point(2) << endl;
        // Color <r g b> (Just say white instead)
        outfile << "255 255 255" << endl;
        // View list: <list_length><camera_index key u v>\n<camera_index key u v>
        // Key is the keypoint # in SIFT, but we just use point number instead.
        // We output these as monocular points because the file format does not
        // support stereo points.
        
        ProjMap &prjs = sbain.tracks[i].projections;
        
        // List length
        outfile << prjs.size() << ' ';
        
        // Output all the tracks as monocular tracks.
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
        {
            Proj &prj = itr->second;
            // y is reversed (-y)
            Node &node = sbain.nodes[prj.ndi];
            
            double cx = node.Kcam(0, 2);
            double cy = node.Kcam(1, 2);
            
            outfile << prj.ndi << ' ' << i << ' ' << prj.kp(0)-cx << ' ' 
                    << -(prj.kp(1)-cy) << ' ';
        }
        
        outfile << endl;
    }

    return 0;
} 

int  sba::ParseBundlerFile(const char *fin,	// input file
		vector< Vector3d, Eigen::aligned_allocator<Vector3d> > &camp, // cam params <f d1 d2>
		vector< Matrix3d, Eigen::aligned_allocator<Matrix3d> > &camR, // cam rotation matrix
		vector< Vector3d, Eigen::aligned_allocator<Vector3d> > &camt, // cam translation
		vector< Vector3d, Eigen::aligned_allocator<Vector3d> > &ptp, // point position
		vector< Vector3i, Eigen::aligned_allocator<Vector3i> > &ptc, // point color
		vector< vector< Vector4d, Eigen::aligned_allocator<Vector4d> > > &ptt // point tracks - each vector is <camera_index u v>
		)
{
    ifstream ifs(fin);
    if (!ifs)
      {
        cout << "Can't open file " << fin << endl;
        return -1;
      }
    ifs.precision(10);


    // read header
    string line;
    if (!getline(ifs,line) || line != "# Bundle file v0.3")
      {
        cout << "Bad header" << endl;
        return -1;
      }
    cout << "Found Bundler 3.0 file" << endl;

    // read number of cameras and points
    int ncams, npts;
    if (!(ifs >> ncams >> npts))
      {
        cout << "Bad header" << endl;  
        return -1;
      }
    cout << "Number of cameras: " << ncams << "  Number of points: " << npts << endl;
    
    cout << "Reading in camera data..." << flush;
    for (int i=0; i<ncams; i++)
      {
        double v1,v2,v3,v4,v5,v6,v7,v8,v9;
        if (!(ifs >> v1 >> v2 >> v3))
	  {
	    cout << "Bad camera params at number " << i << endl;
	    return -1;
	  }
        camp.push_back(Vector3d(v1,v2,v3));

        if (!(ifs >> v1 >> v2 >> v3 >> v4 >> v5 >> v6 >> v7 >> v8 >> v9))
	  {
	    cout << "Bad camera rotation matrix at number " << i << endl;
	    return -1;
	  }
        Matrix3d m;
        m << v1,v2,v3,v4,v5,v6,v7,v8,v9;
        camR.push_back(m);

        if (!(ifs >> v1 >> v2 >> v3))
	  {
	    cout << "Bad camera translation at number " << i << endl;
	    return -1;
	  }
        camt.push_back(Vector3d(v1,v2,v3));
      }
    cout << "done" << endl;

    ptt.resize(npts);

    cout << "Reading in pts data..." << flush;
    for (int i=0; i<npts; i++)
      {
        double v1,v2,v3;
        int i1,i2,i3;

        if (!(ifs >> v1 >> v2 >> v3))
	  {
	    cout << "Bad point position at number " << i << endl;
	    return -1;
	  }
        ptp.push_back(Vector3d(v1,v2,v3));

        if (!(ifs >> i1 >> i2 >> i3))
	  {
	    cout << "Bad point color at number " << i << endl;
	    return -1;
	  }
        ptc.push_back(Vector3i(i1,i2,i3));


        if (!(ifs >> i1))
	  {
	    cout << "Bad track count at number " << i << endl;
	    return -1;
	  }
        int nprjs = i1;

        vector<Vector4d, Eigen::aligned_allocator<Vector4d> > &prjs = ptt[i];
        for (int j=0; j<nprjs; j++)
	  {
	    if (!(ifs >> i1 >> i2 >> v1 >> v2))
	      {
	        cout << "Bad track parameter at number " << i << endl;
	        return -1;
	      }
	    prjs.push_back(Vector4d(i1,i2,v1,v2));
	  }

      } // end of pts loop
    cout << "done" << endl;

    // print some stats
    double nprjs = 0;
    for (int i=0; i<npts; i++)
      nprjs += ptt[i].size();
    cout << "Number of projections: " << (int)nprjs << endl;
    cout << "Average projections per camera: " << nprjs/(double)ncams << endl;
    cout << "Average track length: " << nprjs/(double)npts << endl;
    return 0;
}


// write out the system in an sba (Lourakis) format
// NOTE: Lourakis FAQ is wrong about coordinate systems
//   Cameras are represented by the w2n transform, converted to
//   a quaternion and translation vector
//
void sba::writeLourakisFile(const char *fname, SysSBA& sba)
{
    char name[1024];
    sprintf(name,"%s-cams.txt",fname);
    FILE *fn = fopen(name,"w");
    if (fn == nullptr)
      {
        cout << "[WriteFile] Can't open file " << name << endl;
        return;
      }
    
    // write out initial camera poses
    int ncams = sba.nodes.size();
    for (int i=0; i<ncams; i++)
      {
        Node &nd = sba.nodes[i];

        // Why not just use the Quaternion???
        Quaternion<double> frq(nd.w2n.block<3,3>(0,0)); // rotation matrix of transform
        fprintf(fn,"%f %f %f %f ", frq.w(), frq.x(), frq.y(), frq.z());
        Vector3d tr = nd.w2n.col(3);
        fprintf(fn,"%f %f %f\n", tr[0], tr[1], tr[2]);
      }
    fclose(fn);

    sprintf(name,"%s-pts.txt",fname);
    fn = fopen(name,"w");
    if (fn == nullptr)
      {
        cout << "[WriteFile] Can't open file " << name << endl;
        return;
      }
    
    fprintf(fn,"# X Y Z  nframes  frame0 x0 y0  frame1 x1 y1 ...\n");

    // write out points
    
    for(size_t i=0; i<sba.tracks.size(); i++)
      {
        ProjMap &prjs = sba.tracks[i].projections;
        // Write out point
        Point &pt = sba.tracks[i].point;
        
        fprintf(fn,"%f %f %f  ", pt.x(), pt.y(), pt.z());
        fprintf(fn,"%d  ",(int)prjs.size());
        
        // Write out projections
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;      
            if (!prj.isValid) continue;
            int cami = itr->first;//prj.ndi;
            // NOTE: Lourakis projected y is reversed (???)
            fprintf(fn," %d %f %f ",cami,prj.kp[0],prj.kp[1]);
          }
        fprintf(fn,"\n");
      }

    fclose(fn);

    // write camera calibartion
    sprintf(name,"%s-calib.txt",fname);
    fn = fopen(name,"w");
    if (fn == nullptr)
      {
        cout << "[WriteFile] Can't open file " << name << endl;
        return;
      }
    
    Matrix3d &K = sba.nodes[0].Kcam;
    fprintf(fn,"%f %f %f\n", K(0,0), K(0,1), K(0,2));
    fprintf(fn,"%f %f %f\n", K(1,0), K(1,1), K(1,2));
    fprintf(fn,"%f %f %f\n", K(2,0), K(2,1), K(2,2));

    fclose(fn);
}


// write out the precision matrix
void sba::writeA(const char *fname, SysSBA& sba)
{
    ofstream ofs(fname);
    if (!ofs)
      {
        cout << "Can't open file " << fname << endl;
        return;
      }

    // cameras
    Eigen::IOFormat pfmt(16);
    ofs << sba.A.format(pfmt) << endl;
    ofs.close();
}


// write out the precision matrix for CSparse
void sba::writeSparseA(const char *fname, SysSBA& sba)
{
    char name[1024];
    sprintf(name,"%s-A.tri",fname);

    {
      ofstream ofs(name);
      if (!ofs)
        {
          cout << "Can't open file " << fname << endl;
          return;
        }

      // cameras
      Eigen::IOFormat pfmt(16);

      int nrows = sba.A.rows();
      int ncols = sba.A.cols();
    
      cout << "[WriteSparseA] Size: " << nrows << "x" << ncols << endl;

      // find # nonzeros
      int nnz = 0;
      for (int i=0; i<nrows; i++)
        for (int j=i; j<ncols; j++)
          {
            double a = sba.A(i,j);
            if (a != 0.0) nnz++;
          }

      ofs << nrows << " " << ncols << " " << nnz << " 1" << endl;

      for (int i=0; i<nrows; i++)
        for (int j=i; j<ncols; j++)
          {
            double a = sba.A(i,j);
            if (a != 0.0)
              ofs << i << " " << j << " " << setprecision(16) << a << endl;
          }

      ofs.close();
    }

    sprintf(name,"%s-B.txt",fname);

    {
      ofstream ofs(name);
      if (!ofs)
        {
          cout << "Can't open file " << fname << endl;
          return;
        }

      // cameras
      Eigen::IOFormat pfmt(16);

      int nrows = sba.B.rows();
    
      ofs << nrows << " " << 1 << endl;

      for (int i=0; i<nrows; i++)
        {
          double a = sba.B(i);
          ofs << setprecision(16) << a << endl;
        }
      ofs.close();
    }
}


int sba::readGraphFile(const char *filename, SysSBA& sbaout)
{ 
    // Create vectors to hold the data from the graph file. 
    vector< Vector5d, Eigen::aligned_allocator<Vector5d> > camps;	// cam params <f d1 d2>
    vector< Vector4d, Eigen::aligned_allocator<Vector4d> > camqs;	// cam rotation matrix
    vector< Vector3d, Eigen::aligned_allocator<Vector3d> > camts;	// cam translation
    vector< Vector3d, Eigen::aligned_allocator<Vector3d> > ptps;	// point position
    vector< vector< Vector11d, Eigen::aligned_allocator<Vector11d> > > ptts; // point tracks - each vector is <camera_index kp_idex u v>

    int ret = ParseGraphFile(filename, camps, camqs, camts, ptps, ptts);
    if (ret < 0)
        return -1;
        
    int ncams = camps.size();
    int npts  = ptps.size();
    int nprjs = 0;
    for (int i=0; i<npts; i++)
        nprjs += (int)ptts[i].size();
    //    cout << "Points: " << npts << "  Tracks: " << ptts.size() 
    //         << "  Projections: " << nprjs << endl; 
         
    // cout << "Setting up nodes..." << flush;
    for (int i=0; i<ncams; i++)
    {
        // camera params
        Vector5d &camp = camps[i];
        CamParams cpars = {camp[0],camp[1],camp[2],camp[3],camp[4]}; // set focal length and offsets
	                                                        // note fy is negative...
        //
        // NOTE: not sure how graph files parameterize rotations
	//

        Quaternion<double> frq(camqs[i]); // quaternion coeffs
        if (frq.w() < 0.0)	// w negative, change to positive
        {
            frq.x() = -frq.x();
            frq.y() = -frq.y();
            frq.z() = -frq.z();
            frq.w() = -frq.w();
        }
        
        frq.normalize();

        // translation
        Vector3d &camt = camts[i];
        Vector4d frt;
        frt.head<3>() = camt;
        frt[3] = 1.0;

        Node nd;
        
        sbaout.addNode(frt, frq, cpars);
    }
    // cout << "done" << endl;

    // set up points
    // cout << "Setting up points..." << flush;
    for (int i=0; i<npts; i++)
    {
        // point
        Vector3d &ptp = ptps[i];
        Point pt;
        pt.head<3>() = ptp;
        pt[3] = 1.0;
        sbaout.addPoint(pt);
    }
    // cout << "done" << endl;


    sbaout.useLocalAngles = true;    // use local angles
    sbaout.nFixed = 1;

    // set up projections
    int ntot = 0;
    // cout << "Setting up projections..." << flush;
    for (int i=0; i<npts; i++)
    {
        // track
        vector<Vector11d, Eigen::aligned_allocator<Vector11d> > &ptt = ptts[i];
        int nprjs = ptt.size();
        for (int j=0; j<nprjs; j++)
        {
            // projection
            Vector11d &prj = ptt[j];
            int cami = (int)prj[0];
	    if (cami >= ncams)
	      cout << "*** Cam index exceeds bounds: " << cami << endl;
	    if (prj[4] > 0)	// stereo
	      {
		Vector3d pt = prj.segment<3>(2);
		sbaout.addStereoProj(cami,i,pt); // Monocular projections
	      }
	    else		// mono
	      {
		Vector2d pt = prj.segment<2>(2);
		sbaout.addMonoProj(cami,i,pt); // Monocular projections
	      }

            ntot++;
        }
    }
    // cout << "done" << endl;
    
    return 0;
}


// makes a quaternion from fixed Euler RPY angles
// see the Wikipedia article on Euler anlges
static void make_qrot(double rr, double rp, double ry, Vector4d &v)
{
  double sr = sin(rr/2.0);
  double cr = cos(rr/2.0);
  double sp = sin(rp/2.0);
  double cp = cos(rp/2.0);
  double sy = sin(ry/2.0);
  double cy = cos(ry/2.0);
  v[0] = sr*cp*cy - cr*sp*sy;   // qx
  v[1] = cr*sp*cy + sr*cp*sy;   // qy
  v[2] = cr*cp*sy - sr*sp*cy;   // qz
  v[3] = cr*cp*cy + sr*sp*sy;   // qw
}

int  sba::ParseGraphFile(const char *fin,	// input file
  vector< Vector5d, Eigen::aligned_allocator<Vector5d> > &camp, // cam params <fx fy cx cy>
  vector< Vector4d, Eigen::aligned_allocator<Vector4d> > &camq, // cam rotation quaternion
  vector< Vector3d, Eigen::aligned_allocator<Vector3d> > &camt, // cam translation
  vector< Vector3d, Eigen::aligned_allocator<Vector3d> > &ptp, // point position
  // point tracks - each vector is <camera_index point_index u v d>; 
  // point index is redundant, d is 0 for mono, >0 for stereo
  vector< vector< Vector11d, Eigen::aligned_allocator<Vector11d> > > &ptts 
		)
{
  // input stream
  ifstream ifs(fin);
  if (!ifs)
    {
      cout << "Can't open file " << fin << endl;
      return -1;
    }
  ifs.precision(10);

  // map of node and point indices
  map<int,int> nodemap;
  map<int,int> pointmap;

  // loop over lines
  string line;
  int nline = 0;
  int nid = 0;			// current node id
  int pid = 0;			// current point id
  while (getline(ifs,line))
    {
      nline++;
      stringstream ss(line);    // make a string stream
      string type;
      ss >> type;
      size_t pos = type.find("#");
      if (pos != string::npos)
        continue;               // comment line

      if (type == "VERTEX_SE3" || 
          type == "VERTEX_CAM")    // have a camera node
        {
          int n;
          double tx,ty,tz,qx,qy,qz,qw,fx,fy,cx,cy,bline;
          if (!(ss >> n >> tx >> ty >> tz >> qx >> qy >> qz >> qw >> fx >> fy >>
		cx >> cy >> bline))
            {
              cout << "[ReadSPA] Bad VERTEX_SE3 at line " << nline << endl;
              return -1;
            }
	  nodemap.insert(pair<int,int>(n,nid));
	  nid++;
          camt.push_back(Vector3d(tx,ty,tz));
	  Vector4d v(qx,qy,qz,qw); // use quaternions
	  //          make_qrot(rr,rp,ry,v);
          camq.push_back(v);

	  // fx,fy,cx,cy
	  Vector5d cp;
	  cp << fx,fy,cx,cy,bline;
	  camp.push_back(cp);
        }

      else if (type == "VERTEX_XYZ")    // have a point
        {
          int n;
          double tx,ty,tz;
          if (!(ss >> n >> tx >> ty >> tz))
            {
              cout << "[ReadSPA] Bad VERTEX_XYZ at line " << nline << endl;
              return -1;
            }
	  pointmap.insert(pair<int,int>(n,pid));
	  pid++;
          ptp.push_back(Vector3d(tx,ty,tz));
        }

      else if (type == "EDGE_PROJECT_XYZ" ||
	       type == "EDGE_PROJECT_P2MC" ||
	       type == "EDGE_PROJECT_P2SC") // have an edge
        {
          int n1,n2;
	  double u,v,d;		// projection, including disparity
	  double cv0, cv1, cv2, cv3, cv4, cv5; // covars of point projection, not used
	  cv3 = cv4 =cv5 = 0.0;

          // indices and measurement
	  d = 0;
	  if (type == "EDGE_PROJECT_P2SC")
	    {
	      if (!(ss >> n1 >> n2 >> u >> v >> d >>
		    cv0 >> cv1 >> cv2 >> cv3 >> cv4 >> cv5))
		{
		  cout << "[ReadSPA] Bad EDGE_PROJECT_XYZ at line " << nline << endl;
		  return -1;
		}
	    }
	  else
	    {
	      if (!(ss >> n1 >> n2 >> u >> v >> cv0 >> cv1 >> cv2))
		{
		  cout << "[ReadSPA] Bad EDGE_PROJECT_XYZ at line " << nline << endl;
		  return -1;
		}
	    }

	  // get true indices
	  map<int,int>::iterator it;
	  it = pointmap.find(n1);
	  if (it == pointmap.end())
            {
              cout << "[ReadSPA] Missing point index " << n1 << " at line " << nline << endl;
              return -1;
            }
	  int pi = it->second;
	  if (pi >= (int)ptp.size())
            {
              cout << "[ReadSPA] Point index " << pi << " too large at line " << nline << endl;
              return -1;
            }


	  it = nodemap.find(n2);
	  if (it == nodemap.end())
            {
              cout << "[ReadSPA] Missing camera index " << n2 << " at line " << nline << endl;
              return -1;
            }
	  int ci = it->second;
	  if (ci >= (int)camp.size())
            {
              cout << "[ReadSPA] Camera index " << ci << " too large at line " << nline << endl;
              return -1;
            }

	  
	  // get point track
	  if (ptts.size() < (size_t)pi+1)
	    ptts.resize(pi+1);
	  vector< Vector11d, Eigen::aligned_allocator<Vector11d> > &trk = ptts[pi];
	  Vector11d tv;
	  tv << ci,pi,u,v,d,cv0,cv1,cv2,cv3,cv4,cv5;
	  trk.push_back(tv);
	}

      else
        {
          cout << "[ReadSPA] Undefined type <" << type <<"> at line " << nline << endl;
          return -1;
        }
    }

    // print some stats
    double nprjs = 0;
    int ncams = camp.size();
    int npts = ptp.size();
    for (int i=0; i<npts; i++)
      nprjs += ptts[i].size();
    cout << "Number of cameras: " << ncams << endl;
    cout << "Number of points: " << npts << endl;
    cout << "Number of projections: " << (int)nprjs << endl;
    cout << "Average projections per camera: " << nprjs/(double)ncams << endl;
    cout << "Average track length: " << nprjs/(double)npts << endl;
    return 0;
}


/**
 * \brief Writes out the current SBA system as an ascii graph file
 * suitable to be read in by the Freiburg HChol system.  
 * <mono> is true if only monocular projections are desired
 */

int sba::writeGraphFile(const char *filename, SysSBA& sba, bool mono)
{
    ofstream outfile(filename, ios_base::trunc);
    if (!outfile)
    {
        cout << "Can't open file " << filename << endl;
        return -1;
    }
    
    outfile.precision(5);
    //    outfile.setf(ios_base::scientific);
    outfile.setf(ios_base::fixed);
    
    unsigned int i = 0;
    
    // header, but we skip for now
    //    outfile << "# Bundle file v0.3" << endl;
    
    // Info about each camera
    //   VERTEX_CAM n x y z qx qy qz qw fx fy cx cy baseline
    //   <baseline> is 0 for monocular data
    //   <n> is the camera index, heading at 0
    int ncams = sba.nodes.size();
    for (i = 0; i < (unsigned int)ncams; i++)
    {
      outfile << "VERTEX_CAM" << " ";
      outfile << i << " ";      // node number
      Vector3d trans = sba.nodes[i].trans.head<3>(); // position
      outfile << trans(0) << ' ' << trans(1) << ' ' << trans(2) << ' ';
      Vector4d rot = sba.nodes[i].qrot.coeffs(); // rotation
      outfile << rot(0) << ' ' << rot(1) << ' ' << rot(2) << ' ' << rot(3) << ' ';
      // cam params
      outfile << sba.nodes[i].Kcam(0,0) << ' ' << sba.nodes[i].Kcam(1,1) << ' ' << 
        sba.nodes[i].Kcam(0,2) << ' ' << sba.nodes[i].Kcam(1,2) << ' ' << sba.nodes[i].baseline << endl;
    }
    
    // Info about each point
    //  point indices are sba indices plus ncams (so they're unique in the file)
    //    VERTEX_POINT n x y z
    //  after each point comes the projections
    //    EDGE_PROJECT_P2C pt_ind cam_ind u v
    for (i = 0; i < sba.tracks.size(); i++)
    {
      outfile << "VERTEX_XYZ" << ' ' << ncams+i << ' '; // index
      // World <x y z>
      outfile << sba.tracks[i].point(0) << ' ' << sba.tracks[i].point(1) 
              << ' ' << sba.tracks[i].point(2) << endl;
        
      ProjMap &prjs = sba.tracks[i].projections;
        
      // Output all projections
      //   Mono projections have 0 for the disparity
      for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
        {
	  // TODO: output real covariance, if available
          Proj &prj = itr->second;
	  if (prj.stereo && !mono)
	    {
	      outfile << "EDGE_PROJECT_P2SC "; // stereo edge
	      outfile << ncams+i << ' ' << prj.ndi << ' ' << prj.kp(0) << ' ' 
		      << prj.kp(1) << ' ' << prj.kp(2) << ' ';
	      outfile << "1 0 0 0 1 1" << endl;	// covariance
	    }
	  else
	    {
	      outfile << "EDGE_PROJECT_P2MC "; // mono edge
	      outfile << ncams+i << ' ' << prj.ndi << ' ' << prj.kp(0) << ' ' 
		      << prj.kp(1) << ' ';
	      outfile << "1 0 1" << endl; // covariance
	    }
        }
    }

    return 0;
} 


//
// SPA (3D pose graphs)
//


//
// add a single node to the graph, in the position given by the VERTEX2 entry in the file
//

void 
addnode(SysSPA &spa, int n, 
 std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > ntrans, 	// node translation
 std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > nqrot,	// node rotation
 std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > cind,	// constraint indices
 std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > ctrans,	// constraint local translation 
 std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > cqrot,	// constraint local rotation as quaternion
 std::vector< Eigen::Matrix<double,6,6>, Eigen::aligned_allocator<Eigen::Matrix<double,6,6> > > prec) // constraint covariance
{
  Node nd;

  // rotation
  Quaternion<double> frq;
  frq.coeffs() = nqrot[n];
#if 0
  frq.normalize();
  if (frq.w() <= 0.0) frq.coeffs() = -frq.coeffs();
  nd.qrot = frq.coeffs();
#endif

  // translation
  Vector4d v;
  v.head(3) = ntrans[n];
  v(3) = 1.0;

  spa.addNode(v,frq);

#if 0
  nd.trans = v;
  nd.setTransform();        // set up world2node transform
  nd.setDr(true);

  // add to system
  spa.nodes.push_back(nd);
#endif

  // add in constraints
  for (int i=0; i<(int)ctrans.size(); i++)
    {
      ConP2 con;
      con.ndr = cind[i].x();
      con.nd1 = cind[i].y();

      if ((con.ndr == n && con.nd1 <= n-1) ||
          (con.nd1 == n && con.ndr <= n-1))
        {
	  con.tmean = ctrans[i];
	  Quaternion<double> qr;
	  qr.coeffs() = cqrot[i];
	  qr.normalize();
	  con.qpmean = qr.inverse(); // inverse of the rotation measurement
	  con.prec = prec[i];       // ??? should this be inverted ???

	  // need a boost for noise-offset system
	  //con.prec.block<3,3>(3,3) *= 10.0;
	  spa.p2cons.push_back(con);
	}
    }
}

int sba::readSPAGraphFile(const char *filename, SysSPA& spaout)
{ 
  // Create vectors to hold the data from the graph file. 
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > ntrans; 	// node translation
  std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > nqrot;	// node rotation
  std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > cind;	// constraint indices
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > ctrans;	// constraint local translation 
  std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > cqrot;	// constraint local rotation as quaternion
  std::vector< Eigen::Matrix<double,6,6>, Eigen::aligned_allocator<Eigen::Matrix<double,6,6> > > prec; // constraint covariance

  int ret = ParseSPAGraphFile(filename, ntrans, nqrot, cind, ctrans, cqrot, prec);
  if (ret < 0)
    return -1;
        
  cout << "# [ReadSPAFile] Found " << (int)ntrans.size() << " nodes and " 
       << (int)cind.size() << " constraints" << endl;

  int nnodes = ntrans.size();

  // add in nodes
  for (int i=0; i<nnodes; i++)
    addnode(spaout, i, ntrans, nqrot, cind, ctrans, cqrot, prec);
    
  return 0;
}


// cv is upper triangular
void make_covar(double *cv, Matrix<double,6,6> &m)
{
  m.setZero();

  int i = 0;
  m(0,0) = cv[i++];
  m(0,1) = cv[i++];
  m(0,2) = cv[i++];
  m(0,3) = cv[i++];
  m(0,4) = cv[i++];
  m(0,5) = cv[i++];

  m(1,1) = cv[i++];
  m(1,2) = cv[i++];
  m(1,3) = cv[i++];
  m(1,4) = cv[i++];
  m(1,5) = cv[i++];

  m(2,2) = cv[i++];
  m(2,3) = cv[i++];
  m(2,4) = cv[i++];
  m(2,5) = cv[i++];

  m(3,3) = cv[i++];
  m(3,4) = cv[i++];
  m(3,5) = cv[i++];

  m(4,4) = cv[i++];
  m(4,5) = cv[i++];

  m(5,5) = cv[i++];

  // make symmetric
  Matrix<double,6,6> mt = m.transpose();
  mt.diagonal().setZero();
  m = m+mt;
}

int
sba::ParseSPAGraphFile(const char *fin, // input file
   std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &ntrans, // node translation
   std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &nqrot,  // node rotation
   std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > &cind,   // constraint indices
   std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &ctrans, // constraint local translation 
   std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &cqrot,  // constraint local rotation as quaternion
   std::vector< Eigen::Matrix<double,6,6>, Eigen::aligned_allocator<Eigen::Matrix<double,6,6> > > &prec) // constraint covariance
{
  ifstream ifs(fin);
  if (!ifs)
    {
      cout << "Can't open file " << fin << endl;
      return -1;
    }
  ifs.precision(10);

  // loop over lines
  string line;
  int nline = 0;
  bool first = true;
  while (getline(ifs,line))
    {
      nline++;
      stringstream ss(line);    // make a string stream
      string type;
      ss >> type;
      size_t pos = type.find("#");
      if (pos != string::npos)
        continue;               // comment line

      if (type == "VERTEX_SE3")    // have a vertex
        {
          int n;
          double tx,ty,tz,rr,rp,ry;
          if (!(ss >> n >> tx >> ty >> tz >> rr >> rp >> ry))
            {
              cout << "[ReadSPA] Bad VERTEX_SE3 at line " << nline << endl;
              return -1;
            }
          ntrans.push_back(Vector3d(tx,ty,tz));
          Vector4d v;
          make_qrot(rr,rp,ry,v);
          nqrot.push_back(v);
        }

      if (type == "EDGE_SE3_SE3")      // have an edge
        {
          int n1,n2;
          double tx,ty,tz,rr,rp,ry;
          double cv[21];

          // indices and measurement
          if (!(ss >> n1 >> n2 >> tx >> ty >> tz >> rr >> rp >> ry))
            {
              cout << "[ReadSPA] Bad EDGE_SE3_SE3 at line " << nline << endl;
              return -1;
            }
          cind.push_back(Vector2i(n1,n2));
          ctrans.push_back(Vector3d(tx,ty,tz));
          Vector4d v;
          make_qrot(rr,rp,ry,v);
          cqrot.push_back(v);

          // covar
          if (!(ss >> cv[0] >> cv[1] >> cv[2] >> cv[3] >> cv[4] 
                >> cv[5] >> cv[6] >> cv[7] >> cv[8] >> cv[9] 
                >> cv[10] >> cv[11] >> cv[12] >> cv[13] >> cv[14] 
                >> cv[15] >> cv[16] >> cv[17] >> cv[18] >> cv[19] >> cv[20]))
            {
              cout << "[ReadSPA] Bad EDGE_SE3_SE3 at line " << nline << endl;
              return -1;
            }
          Matrix<double,6,6> m;
          make_covar(cv,m);
          if (first)
            {
              //cout << endl;
              //for (int j=0; j<21; j++);
                //cout << cv[j] << " ";
              //cout << endl << endl << << m << endl;
              first = false;
            }
          prec.push_back(m);
        }

    }
  return 0;
}
