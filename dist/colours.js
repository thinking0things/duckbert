export function robotColour(name,palette='bordeaux') {
  if(/^OLED_eye_/.test(name))return '#ffffff';
  if(name==='OLED_screen')return '#050608';
  if(/^[LR]_(hip_roll|hip_pitch|knee)$/.test(name))return '#1765d1';
  if(palette==='arctic') {
    if(name.startsWith('07_'))return '#ff842b';
    if(/^(03_|04_|15_)/.test(name)||name.endsWith('_stock_horn'))return '#28cbd7';
    return '#f2f4f5';
  }
  return '#800020';
}
