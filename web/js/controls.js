// Parameter panel, generated entirely from the schema the engine serves.
// Nothing about the model is duplicated here: adding a parameter in
// params.py makes a control appear.

const OPEN_BY_DEFAULT = new Set(['Payload', 'Propulsion', 'Battery', 'Mission']);

export class Controls {
  constructor(container, schema, groups, defaults, onChange) {
    this.el = container;
    this.schema = schema;
    this.groups = groups;
    this.defaults = { ...defaults };
    this.values = { ...defaults };
    this.onChange = onChange;
    this.showAdvanced = false;
    this.nodes = new Map();
    this.render();
  }

  render() {
    this.el.innerHTML = '';
    const adv = document.createElement('div');
    adv.style.cssText = 'padding:9px 14px;border-bottom:1px solid var(--line)';
    adv.innerHTML = `<label style="color:var(--muted);font-size:11px;cursor:pointer">
      <input type="checkbox" id="advtoggle" ${this.showAdvanced ? 'checked' : ''}
      style="accent-color:var(--accent)"> show advanced assumptions</label>`;
    adv.querySelector('#advtoggle').onchange = (e) => {
      this.showAdvanced = e.target.checked;
      this.render();
    };
    this.el.appendChild(adv);

    for (const g of this.groups) {
      const items = this.schema.filter((s) => s.group === g &&
        (this.showAdvanced || !s.advanced));
      if (!items.length) continue;
      const d = document.createElement('details');
      d.className = 'group';
      d.open = OPEN_BY_DEFAULT.has(g);
      const s = document.createElement('summary');
      s.textContent = g;
      d.appendChild(s);
      const body = document.createElement('div');
      body.className = 'body';
      for (const it of items) body.appendChild(this.control(it));
      d.appendChild(body);
      this.el.appendChild(d);
    }
  }

  control(spec) {
    const wrap = document.createElement('div');
    wrap.className = 'ctl';
    const v = this.values[spec.key];

    if (spec.boolean) {
      wrap.className = 'ctl bool';
      const id = 'c_' + spec.key;
      wrap.innerHTML = `<input type="checkbox" id="${id}" ${Number(v) ? 'checked' : ''}>
        <label for="${id}" title="${(spec.help || '').replace(/"/g, "'")}">${spec.label}</label>`;
      const cb = wrap.querySelector('input');
      cb.onchange = () => this.set(spec.key, cb.checked ? 1 : 0);
      this.nodes.set(spec.key, { wrap, box: cb });
      this.mark(spec.key);
      return wrap;
    }

    const step = spec.step || (spec.hi - spec.lo) / 400;
    wrap.innerHTML = `
      <div class="top">
        <label title="${(spec.help || '').replace(/"/g, "'")}">${spec.label}</label>
        <span><input class="val" type="text" value="${fmtNum(v)}"> <span class="unit">${spec.unit === '-' ? '' : spec.unit}</span></span>
      </div>
      <input type="range" min="${spec.lo}" max="${spec.hi}" step="${step}" value="${v}">`;
    const range = wrap.querySelector('input[type=range]');
    const text = wrap.querySelector('.val');

    range.oninput = () => {
      const nv = spec.integer ? Math.round(+range.value) : +range.value;
      text.value = fmtNum(nv);
      this.set(spec.key, nv, true);
    };
    range.onchange = () => this.set(spec.key, spec.integer ? Math.round(+range.value) : +range.value);
    text.onchange = () => {
      const nv = parseFloat(text.value);
      if (!isFinite(nv)) { text.value = fmtNum(this.values[spec.key]); return; }
      const c = Math.min(Math.max(nv, spec.lo), spec.hi);
      range.value = c;
      text.value = fmtNum(c);
      this.set(spec.key, c);
    };
    this.nodes.set(spec.key, { wrap, range, text, spec });
    this.mark(spec.key);
    return wrap;
  }

  mark(key) {
    const n = this.nodes.get(key);
    if (!n) return;
    n.wrap.classList.toggle('changed', this.values[key] !== this.defaults[key]);
  }

  set(key, value, live = false) {
    this.values[key] = value;
    this.mark(key);
    this.onChange(this.values, key, live);
  }

  setAll(obj) {
    for (const [k, v] of Object.entries(obj)) {
      if (!(k in this.values)) continue;
      this.values[k] = v;
      const n = this.nodes.get(k);
      if (!n) continue;
      if (n.box) n.box.checked = !!Number(v);
      else { n.range.value = v; n.text.value = fmtNum(v); }
      this.mark(k);
    }
    this.onChange(this.values, null, false);
  }

  reset() { this.setAll(this.defaults); }
}

function fmtNum(v) {
  if (typeof v !== 'number' || !isFinite(v)) return String(v);
  if (Number.isInteger(v)) return String(v);
  const a = Math.abs(v);
  if (a >= 1e5 || a < 1e-3) return v.toExponential(2);
  return String(Number(v.toPrecision(4)));
}
