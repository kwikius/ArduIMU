


module shape1(){
   w = sqrt(2);
   l = w + 2;

   hull(){
   cube([l,w,w],center = true);

   cube([w,l,w],center = true);

   cube([w,w,l],center = true);
   }
}

//shape1();

function mirror_y (v) = [v[0],-v[1],v[2]];
module shape2()
{
  root_chord =8;
  root_thickness = 1;
  tip_chord = 4;
  tip_thickness = root_thickness * tip_chord/ root_chord;
  half_span = 10;
  le_sweep = 8;

  root_le = [0,0,0];
  root_le_idx = 0;

  root_te = [root_chord,0,0];
  root_te_idx = root_le_idx + 1;

  tip_le = [le_sweep,half_span,0];
  tip_le_idx = root_te_idx + 1;

  tip_te  = [le_sweep+tip_chord,half_span,0];
  tip_te_idx = tip_le_idx + 1;

  root_thickest = (root_le * 2 + root_te)/3 + [0,0, root_thickness];
  root_thickest_idx = tip_te_idx +1;

  tip_thickest = (tip_le * 2 + tip_te)/3 + [0,0, tip_thickness];
  tip_thickest_idx = root_thickest_idx + 1;

  tip_le_m_idx = tip_thickest_idx +1;
  tip_te_m_idx = tip_le_m_idx +1;
  tip_thickest_m_idx = tip_te_m_idx +1;

  pts = [
      root_le  ,   // 0         
      root_te,     // 1
      tip_le,      // 2
      tip_te,      // 3
      root_thickest, /// 4
      tip_thickest,   // 5
      mirror_y(tip_le), //6
      mirror_y(tip_te) , //7
      mirror_y(tip_thickest) 
  ];

  fcs = [
      [tip_thickest_m_idx,tip_le_m_idx,root_le_idx,root_thickest_idx]
      ,[root_thickest_idx,root_le_idx,tip_le_idx,tip_thickest_idx]
      ,[root_thickest_idx,tip_thickest_idx,tip_te_idx,root_te_idx]
      ,[tip_thickest_m_idx,root_thickest_idx,root_te_idx,tip_te_m_idx]
      ,[tip_te_m_idx,root_te_idx,tip_te_idx,tip_le_idx,root_le_idx,tip_le_m_idx],
      ,[tip_le_idx,tip_thickest_idx,tip_te_idx],
     ,[tip_te_m_idx,tip_thickest_m_idx,tip_le_m_idx]
   ];   

   polyhedron(pts,fcs,10);
}

shape2();



