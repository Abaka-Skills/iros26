/* 我们的水印（visitor.html 与 pickandplay.html 共用）：Abaka logo 的品牌橙与几何无衬线，几行字各自缩放到同一宽度，
   像一个标志。WM: {color, lines, tracking, lineHeight}。返回贴图，由页面印在桌下的地面上。 */
import * as THREE from 'three';

export function watermarkTexture(WM) {
  const lines = WM.lines, color = WM.color;
  const c = document.createElement('canvas');
  c.width = 2048;
  c.height = 16;                                     // 先量字，量完才知道该多高
  const font = px => `800 ${px}px "Futura", "Avenir Next", "Century Gothic", ` +
                     `"Poppins", "Helvetica Neue", system-ui, sans-serif`;
  const setup = g => {
    g.textAlign = 'left';
    g.textBaseline = 'middle';
    g.fillStyle = color;
    try { g.letterSpacing = WM.tracking + 'px'; } catch {}
  };
  const pad = 20;                                    // 留边，否则末字母会被画布切掉
  const avail = c.width - pad * 2;

  let g = c.getContext('2d');
  setup(g);
  /* 每行各自缩放到同一宽度：短的字就大，整块像一个标志 */
  const sizes = lines.map(t => {
    g.font = font(400);
    const m = g.measureText(t);
    return Math.floor(400 * avail / Math.max(m.width, m.actualBoundingBoxRight || 0));
  });
  const rows = sizes.map(px => Math.round(px * WM.lineHeight));
  c.height = rows.reduce((a, b) => a + b, 0);        // 改尺寸会重置 context，要重新设一遍
  g = c.getContext('2d');
  setup(g);
  let y = 0;
  lines.forEach((t, i) => {
    g.font = font(sizes[i]);
    g.fillText(t, pad, y + rows[i] / 2);
    y += rows[i];
  });
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}
