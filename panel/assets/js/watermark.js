/* 我们的水印（visitor.html 与 pickandplay.html 共用）：Abaka logo 的品牌橙与几何无衬线，几行字各自缩放到同一宽度，
   像一个标志。WM: {color, lines, tracking, lineHeight}。一行可以是字符串（撑满整块宽度），也可以是
   {text, fit（占整块宽度的比例，默认 1）, weight（字重，默认 800）, tracking（字距，默认 WM.tracking）}：
   说明文字这类长句用较细的字重，品牌这类短词只占一部分宽度。返回贴图，由页面印在桌下的地面上。 */
import * as THREE from 'three';

export function watermarkTexture(WM) {
  const lines = WM.lines.map(l => ({ fit: 1, weight: 800, tracking: WM.tracking, ...(typeof l === 'string' ? { text: l } : l) }));
  const color = WM.color;
  const c = document.createElement('canvas');
  c.width = 2048;
  c.height = 16;                                     // 先量字，量完才知道该多高
  const font = (px, weight) => `${weight} ${px}px "Futura", "Avenir Next", "Century Gothic", ` +
                               `"Poppins", "Helvetica Neue", system-ui, sans-serif`;
  const setup = (g, l) => {
    g.textAlign = 'left';
    g.textBaseline = 'middle';
    g.fillStyle = color;
    try { g.letterSpacing = l.tracking + 'px'; } catch {}
  };
  const pad = 20;                                    // 留边，否则末字母会被画布切掉
  const avail = c.width - pad * 2;

  let g = c.getContext('2d');
  /* 每行各自缩放到它的宽度（整块宽度乘 fit）：短的字就大，整块像一个标志。字距是固定像素，不随字号缩放：
     先不加字距量字形，缩放字形，再把每个字后面的字距加回去 */
  const sizes = lines.map(l => {
    setup(g, { ...l, tracking: 0 });
    g.font = font(400, l.weight);
    const m = g.measureText(l.text);
    const glyphs = Math.max(m.width, m.actualBoundingBoxRight || 0);
    return Math.floor(400 * (avail * l.fit - [...l.text].length * l.tracking) / glyphs);
  });
  const rows = sizes.map(px => Math.round(px * WM.lineHeight));
  c.height = rows.reduce((a, b) => a + b, 0);        // 改尺寸会重置 context，要重新设一遍
  g = c.getContext('2d');
  let y = 0;
  lines.forEach((l, i) => {
    setup(g, l);
    g.font = font(sizes[i], l.weight);
    g.fillText(l.text, pad, y + rows[i] / 2);
    y += rows[i];
  });
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}
